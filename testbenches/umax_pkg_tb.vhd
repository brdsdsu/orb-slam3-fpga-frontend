library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.umax_pkg.all;

-- Smoke test for umax_pkg: prints U_MAX(0..HALF_PATCH_SIZE) so the
-- elaboration-time table can be eyeballed against OpenCV's u_max[]. Reports
-- only -- NOT self-checking (no pass/fail assertion).
entity umax_pkg_tb is
end entity;

architecture sim of umax_pkg_tb is
begin
    process
    begin
        for v in 0 to HALF_PATCH_SIZE loop
            report "u_max(" & to_string(v) & ") = " & to_string(U_MAX(v));
        end loop;
        wait;
    end process;
end architecture;