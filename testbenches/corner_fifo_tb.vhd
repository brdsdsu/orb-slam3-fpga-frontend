library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

use work.feature_pkg.all;
use work.umax_pkg.all;

-- Testbench for corner_fifo (synchronous show-ahead FIFO with 31x31 patch payload).
--
-- Directed test, no external vectors: each entry's header fields and patch are
-- derived from an integer id via make_patch / exp_* so the checker recomputes
-- the expected values rather than storing them. Tests T1..T9 cover reset, half
-- fill + count, in-order drain, fill-to-full + tail-drop with drop_count,
-- read-from-empty no-op, simultaneous read/write, soft reset, and power-of-2
-- pointer wraparound. Self-checking; fails (severity failure) on any mismatch.
--
-- Patch storage is circle-masked (umax_pkg.in_circle): only in-circle pixels
-- round-trip; out-of-circle pixels read back as ZEROS by design. The patch
-- checks below pick in-circle probes and verify the zero property explicitly.
-- o_prog_full (backpressure, PROG_FULL_GAP=32 default) is checked at half
-- fill (deasserted), full (asserted), and after drain (deasserted).
entity corner_fifo_tb is
end entity;

architecture sim of corner_fifo_tb is

    constant CLK_PERIOD : time    := 4 ns;
    constant FIFO_DEPTH : natural := 256;   -- entity default (TOP ships 512); drop to e.g. 16 for faster sim
    constant COUNT_WIDTH : natural := natural(ceil(log2(real(FIFO_DEPTH + 1))));

    -- DUT signals
    signal clk            : std_logic := '0';
    signal rst            : std_logic := '1';
    signal wr_en          : std_logic := '0';
    signal wr_x           : coord_t   := (others => '0');
    signal wr_y           : coord_t   := (others => '0');
    signal wr_score       : pixel_t   := (others => '0');
    signal wr_is_brighter : std_logic := '0';
    signal wr_patch       : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
    signal rd_en          : std_logic := '0';
    signal rd_x           : coord_t;
    signal rd_y           : coord_t;
    signal rd_score       : pixel_t;
    signal rd_is_brighter : std_logic;
    signal rd_valid       : std_logic;
    signal rd_patch       : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
    signal o_empty        : std_logic;
    signal o_full         : std_logic;
    signal o_prog_full    : std_logic;
    signal o_count        : unsigned(COUNT_WIDTH-1 downto 0);
    signal o_drop_count   : unsigned(31 downto 0);

    -- Tracking
    signal errors : natural := 0;
    signal done   : boolean := false;

    -- Build a patch where pixel(r, c) = (id + r + c) mod 256
    -- Lets the checker recompute the expected value from the entry id alone.
    function make_patch(id : natural) return patch_arr_t is
        variable p : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
    begin
        for r in 0 to PATCH_SIZE-1 loop
            for c in 0 to PATCH_SIZE-1 loop
                p(r, c) := to_unsigned((id + r + c) mod 256, PIXEL_WIDTH);
            end loop;
        end loop;
        return p;
    end function;

    -- Derive expected header fields from entry id
    function exp_x(id : natural)     return coord_t is
    begin return to_unsigned(id mod 2048, COORD_WIDTH); end function;

    function exp_y(id : natural)     return coord_t is
    begin return to_unsigned((id * 7) mod 2048, COORD_WIDTH); end function;

    function exp_score(id : natural) return pixel_t is
    begin return to_unsigned((id * 13) mod 256, PIXEL_WIDTH); end function;

    function exp_brighter(id : natural) return std_logic is
    begin
        if (id mod 2) = 0 then return '1'; else return '0'; end if;
    end function;

begin

    dut : entity work.corner_fifo
        generic map (
            FIFO_DEPTH => FIFO_DEPTH
        )
        port map (
            clk            => clk,
            rst            => rst,
            wr_en          => wr_en,
            wr_x           => wr_x,
            wr_y           => wr_y,
            wr_score       => wr_score,
            wr_is_brighter => wr_is_brighter,
            wr_patch       => wr_patch,
            rd_en          => rd_en,
            rd_x           => rd_x,
            rd_y           => rd_y,
            rd_score       => rd_score,
            rd_is_brighter => rd_is_brighter,
            rd_valid       => rd_valid,
            rd_patch       => rd_patch,
            o_empty        => o_empty,
            o_full         => o_full,
            o_prog_full    => o_prog_full,
            o_count        => o_count,
            o_drop_count   => o_drop_count
        );

    clk <= not clk after CLK_PERIOD/2 when not done else '0';

    test : process

        -- Helpers to make the test sequence read clearly
        procedure pulse_clk is
        begin
            wait until rising_edge(clk);
			wait for 1 ps;
        end procedure;

        procedure push_entry(id : natural) is
        begin
            wr_x           <= exp_x(id);
            wr_y           <= exp_y(id);
            wr_score       <= exp_score(id);
            wr_is_brighter <= exp_brighter(id);
            wr_patch       <= make_patch(id);
            wr_en          <= '1';
            pulse_clk;
            wr_en <= '0';
        end procedure;

        -- Issue read, wait one cycle for BRAM read latency, then check
        procedure pop_and_check(id : natural; tag : string) is
            variable patch_mismatch : boolean := false;
            variable expected_patch : patch_arr_t(0 to PATCH_SIZE-1, 0 to PATCH_SIZE-1);
        begin
            -- show-ahead: data already at rd_* outputs (registered from previous cycle)
            if rd_x /= exp_x(id) then
                report tag & ": rd_x mismatch for entry " & to_string(id) &
                       ": got " & to_string(to_integer(rd_x)) &
                       ", expected " & to_string(to_integer(exp_x(id)))
                    severity error;
                errors <= errors + 1;
            end if;
            if rd_y /= exp_y(id) then
                report tag & ": rd_y mismatch for entry " & to_string(id)
                    severity error;
                errors <= errors + 1;
            end if;
            if rd_score /= exp_score(id) then
                report tag & ": rd_score mismatch for entry " & to_string(id)
                    severity error;
                errors <= errors + 1;
            end if;
            if rd_is_brighter /= exp_brighter(id) then
                report tag & ": rd_is_brighter mismatch for entry " & to_string(id)
                    severity error;
                errors <= errors + 1;
            end if;

            -- Check a few IN-CIRCLE patch pixels (not all to keep output
            -- readable): top/bottom row centers, center row ends, center.
            expected_patch := make_patch(id);
            if rd_patch(0, 15)  /= expected_patch(0, 15) or
               rd_patch(15, 0)  /= expected_patch(15, 0) or
               rd_patch(15, 15) /= expected_patch(15, 15) or
               rd_patch(15, 30) /= expected_patch(15, 30) or
               rd_patch(30, 15) /= expected_patch(30, 15) then
                report tag & ": rd_patch mismatch for entry " & to_string(id)
                    severity error;
                errors <= errors + 1;
            end if;

            -- Out-of-circle pixels are not stored and must read back as zeros
            if rd_patch(0, 0)  /= to_unsigned(0, PIXEL_WIDTH) or
               rd_patch(30, 30) /= to_unsigned(0, PIXEL_WIDTH) then
                report tag & ": out-of-circle pixel not zero for entry " & to_string(id)
                    severity error;
                errors <= errors + 1;
            end if;

			-- Advance to next entry
			rd_en <= '1';
			pulse_clk;
			rd_en <= '0';
			--pulse_clk;   -- Wait for BRAM read of new head to settle
        end procedure;

        procedure check_status(
            tag : string;
            exp_empty : std_logic;
            exp_full  : std_logic;
            exp_count : natural;
            exp_drops : natural := 0
        ) is
        begin
            if o_empty /= exp_empty then
                report tag & ": o_empty = " & to_string(o_empty) &
                       ", expected " & to_string(exp_empty)
                    severity error;
                errors <= errors + 1;
            end if;
            if o_full /= exp_full then
                report tag & ": o_full = " & to_string(o_full) &
                       ", expected " & to_string(exp_full)
                    severity error;
                errors <= errors + 1;
            end if;
            if to_integer(o_count) /= exp_count then
                report tag & ": o_count = " & to_string(to_integer(o_count)) &
                       ", expected " & to_string(exp_count)
                    severity error;
                errors <= errors + 1;
            end if;
            if to_integer(o_drop_count) /= exp_drops then
                report tag & ": o_drop_count = " &
                       to_string(to_integer(o_drop_count)) &
                       ", expected " & to_string(exp_drops)
                    severity error;
                errors <= errors + 1;
            end if;
        end procedure;

    begin
        -- ============================================================
        -- Reset
        -- ============================================================
        rst <= '1';
        wait for 5 * CLK_PERIOD;
        rst <= '0';
        pulse_clk;

        report "Test 1: empty after reset";
        check_status("T1", exp_empty => '1', exp_full => '0', exp_count => 0);

        -- ============================================================
        -- Test 2: push half the FIFO, verify count
        -- ============================================================
        report "Test 2: push " & to_string(FIFO_DEPTH/2) & " entries";
        for i in 0 to FIFO_DEPTH/2 - 1 loop
            push_entry(i);
        end loop;
        -- Wait one cycle for BRAM read to settle (show-ahead has 1-cycle latency)
        pulse_clk;
        check_status("T2", exp_empty => '0', exp_full => '0', exp_count => FIFO_DEPTH/2);
        if o_prog_full /= '0' then
            report "T2: o_prog_full asserted at half fill" severity error;
            errors <= errors + 1;
        end if;

        -- ============================================================
        -- Test 3: pop all entries in order
        -- ============================================================
        report "Test 3: pop and verify order";
        for i in 0 to FIFO_DEPTH/2 - 1 loop
            pop_and_check(i, "T3");
        end loop;
        pulse_clk;
        check_status("T3 final", exp_empty => '1', exp_full => '0', exp_count => 0);

        -- ============================================================
        -- Test 4: fill FIFO to full, verify full flag
        -- ============================================================
        report "Test 4: fill to full";
        for i in 100 to 100 + FIFO_DEPTH - 1 loop
            push_entry(i);
        end loop;
        pulse_clk;
        check_status("T4", exp_empty => '0', exp_full => '1', exp_count => FIFO_DEPTH);
        if o_prog_full /= '1' then
            report "T4: o_prog_full not asserted at full" severity error;
            errors <= errors + 1;
        end if;

        -- ============================================================
        -- Test 5: push when full, verify drop count and no corruption
        -- ============================================================
        report "Test 5: 5 pushes when full";
        for i in 200 to 204 loop
            push_entry(i);   -- all should be dropped
        end loop;
        pulse_clk;
        check_status("T5", exp_empty => '0', exp_full => '1',
                     exp_count => FIFO_DEPTH, exp_drops => 5);

        -- Pop all entries and verify they're the ORIGINAL push, not the dropped ones
        report "Test 5b: verify dropped pushes did not corrupt FIFO";
        for i in 100 to 100 + FIFO_DEPTH - 1 loop
            pop_and_check(i, "T5b");
        end loop;
        pulse_clk;
        check_status("T5b final", exp_empty => '1', exp_full => '0',
                     exp_count => 0, exp_drops => 5);

        -- ============================================================
        -- Test 6: read from empty (no-op)
        -- ============================================================
        report "Test 6: read from empty";
        rd_en <= '1';
        pulse_clk;
        rd_en <= '0';
        pulse_clk;
        check_status("T6", exp_empty => '1', exp_full => '0',
                     exp_count => 0, exp_drops => 5);

        -- ============================================================
        -- Test 7: simultaneous read and write
        -- Set up: push 4 entries, then simultaneously push and pop
        -- ============================================================
        report "Test 7: simultaneous read/write";
        for i in 300 to 303 loop
            push_entry(i);
        end loop;
        pulse_clk;
        check_status("T7 setup", exp_empty => '0', exp_full => '0',
                     exp_count => 4, exp_drops => 5);

        -- One cycle of simultaneous push (id=304) and pop (id=300)
        wr_x           <= exp_x(304);
        wr_y           <= exp_y(304);
        wr_score       <= exp_score(304);
        wr_is_brighter <= exp_brighter(304);
        wr_patch       <= make_patch(304);
        wr_en          <= '1';
        rd_en          <= '1';
        -- We're about to read entry 300 (the head)
        if rd_x /= exp_x(300) then
            report "T7: head should be id=300 before simultaneous op" severity error;
            errors <= errors + 1;
        end if;
        pulse_clk;
        wr_en <= '0';
        rd_en <= '0';
        pulse_clk;

        -- Count should be unchanged (one in, one out)
        check_status("T7 after simul", exp_empty => '0', exp_full => '0',
                     exp_count => 4, exp_drops => 5);

        -- Pop the remaining: 301, 302, 303, 304
        for i in 301 to 304 loop
            pop_and_check(i, "T7 final");
        end loop;
        pulse_clk;
        check_status("T7 done", exp_empty => '1', exp_full => '0',
                     exp_count => 0, exp_drops => 5);

        -- ============================================================
        -- Test 8: reset behavior
        -- ============================================================
        report "Test 8: reset clears state";
        -- Fill partway
        for i in 400 to 405 loop
            push_entry(i);
        end loop;
        pulse_clk;
        check_status("T8 before reset", exp_empty => '0', exp_full => '0',
                     exp_count => 6, exp_drops => 5);

        rst <= '1';
        wait for 3 * CLK_PERIOD;
        rst <= '0';
        pulse_clk;
        check_status("T8 after reset", exp_empty => '1', exp_full => '0',
                     exp_count => 0, exp_drops => 0);
					 
		-- ============================================================
		-- Test 9: Full-depth fill, drain, and address wraparound
		-- Pushes FIFO_DEPTH entries, drains them, then repeats with a
		-- different ID range. The second push/drain forces the address
		-- counters to wrap, exercising the power-of-2 modular arithmetic.
		-- ============================================================
		report "Test 9: full-depth fill/drain with wraparound";

		-- First pass: fill completely
		for i in 500 to 500 + FIFO_DEPTH - 1 loop
			push_entry(i);
		end loop;
		pulse_clk;
		check_status("T9 first fill", exp_empty => '0', exp_full => '1',
					 exp_count => FIFO_DEPTH, exp_drops => 0);

		-- Drain completely, verifying every entry
		for i in 500 to 500 + FIFO_DEPTH - 1 loop
			pop_and_check(i, "T9 first drain");
		end loop;
		pulse_clk;
		check_status("T9 first drain done", exp_empty => '1', exp_full => '0',
					 exp_count => 0, exp_drops => 0);
		if o_prog_full /= '0' then
			report "T9: o_prog_full still asserted after drain" severity error;
			errors <= errors + 1;
		end if;

		-- Second pass: fill again with different IDs
		-- This forces the address pointers to wrap past their max value,
		-- exercising the natural modular arithmetic of power-of-2 depth.
		for i in 1000 to 1000 + FIFO_DEPTH - 1 loop
			push_entry(i);
		end loop;
		pulse_clk;
		check_status("T9 second fill", exp_empty => '0', exp_full => '1',
					 exp_count => FIFO_DEPTH, exp_drops => 0);

		-- Drain again, verifying order preservation across the wrap
		for i in 1000 to 1000 + FIFO_DEPTH - 1 loop
			pop_and_check(i, "T9 second drain");
		end loop;
		pulse_clk;
		check_status("T9 second drain done", exp_empty => '1', exp_full => '0',
					 exp_count => 0, exp_drops => 0);

        -- ============================================================
        -- Done
        -- ============================================================
        wait for 5 * CLK_PERIOD;
        report "Total errors: " & to_string(errors);
        assert errors = 0 report "TEST FAILED" severity failure;
        report "TEST PASSED" severity note;
        done <= true;
        wait;
    end process;

end architecture;