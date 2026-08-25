--------------------------------------------------------------------------------
-- TOP_vivado_wrapper_vhdl_2002.vhd
-- VHDL-93/2002 packaging wrapper (entity orb_feature_top) around the VHDL-2008
-- TOP entity.
--
-- WHY THIS FILE EXISTS:
--   UG1118 (https://docs.amd.com/r/en-US/ug1118-vivado-creating-packaging-custom-ip)
--   requires the *designated top-level* file of a custom IP to be plain
--   Verilog or VHDL (93/2002), not VHDL-2008. This wrapper presents a clean
--   VHDL-93 entity (orb_feature_top) for the IP packager to introspect, while
--   the VHDL-2008 implementation (TOP and all sub-modules) stays one level
--   down, completely unchanged. It only forwards generics and ports 1:1.
--
--   File types when packaging:
--     TOP_vivado_wrapper_vhdl_2002.vhd .. VHDL (93/2002)  <- designated top
--     feature_pkg.vhd ................... VHDL (93/2002)
--     TOP.vhd + rest .................... VHDL 2008
--------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

use work.feature_pkg.all;   -- PIXEL_WIDTH (top-level port widths only)

entity orb_feature_top is
	generic (
		G_KP_WIDTH         : positive := 128;
		G_KP_FIFO_DEPTH    : positive := 1024;
		G_IDLE_GAP         : positive := 512;
		G_MAX_WIDTH        : positive := 752;
		G_THRESH_PERM      : natural  := 7;
		G_THRESH_STRICT    : natural  := 20;
		G_SCORE_TYPE       : natural  := 1;    -- 0 = HARRIS_SCORE, 1 = FAST_SCORE
		G_CORE_FIFO_DEPTH  : positive := 512;
		G_CORE_URAM_COLS   : positive := 32;
		G_PROG_FULL_GAP    : positive := 530;
		C_S_AXI_DATA_WIDTH : integer  := 32;
		C_S_AXI_ADDR_WIDTH : integer  := 6
	);
	port (
		aclk          : in  std_logic;
		aresetn       : in  std_logic;

		-- AXI4-Stream slave : pixel input
		s_axis_tdata  : in  std_logic_vector(PIXEL_WIDTH-1 downto 0);
		s_axis_tkeep  : in  std_logic_vector((PIXEL_WIDTH/8)-1 downto 0);
		s_axis_tvalid : in  std_logic;
		s_axis_tready : out std_logic;
		s_axis_tlast  : in  std_logic;

		-- AXI4-Stream master : keypoint output
		m_axis_tdata  : out std_logic_vector(G_KP_WIDTH-1 downto 0);
		m_axis_tkeep  : out std_logic_vector((G_KP_WIDTH/8)-1 downto 0);
		m_axis_tvalid : out std_logic;
		m_axis_tready : in  std_logic;
		m_axis_tlast  : out std_logic;

		-- AXI4-Lite slave : control / status
		s_axi_awaddr  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		s_axi_awprot  : in  std_logic_vector(2 downto 0);
		s_axi_awvalid : in  std_logic;
		s_axi_awready : out std_logic;
		s_axi_wdata   : in  std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		s_axi_wstrb   : in  std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
		s_axi_wvalid  : in  std_logic;
		s_axi_wready  : out std_logic;
		s_axi_bresp   : out std_logic_vector(1 downto 0);
		s_axi_bvalid  : out std_logic;
		s_axi_bready  : in  std_logic;
		s_axi_araddr  : in  std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
		s_axi_arprot  : in  std_logic_vector(2 downto 0);
		s_axi_arvalid : in  std_logic;
		s_axi_arready : out std_logic;
		s_axi_rdata   : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
		s_axi_rresp   : out std_logic_vector(1 downto 0);
		s_axi_rvalid  : out std_logic;
		s_axi_rready  : in  std_logic
	);
end entity orb_feature_top;

architecture rtl of orb_feature_top is
begin

	u_top : entity work.TOP
		generic map (
			G_KP_WIDTH         => G_KP_WIDTH,
			G_KP_FIFO_DEPTH    => G_KP_FIFO_DEPTH,
			G_IDLE_GAP         => G_IDLE_GAP,
			G_MAX_WIDTH        => G_MAX_WIDTH,
			G_THRESH_PERM      => G_THRESH_PERM,
			G_THRESH_STRICT    => G_THRESH_STRICT,
			G_SCORE_TYPE       => G_SCORE_TYPE,
			G_CORE_FIFO_DEPTH  => G_CORE_FIFO_DEPTH,
			G_CORE_URAM_COLS   => G_CORE_URAM_COLS,
			G_PROG_FULL_GAP    => G_PROG_FULL_GAP,
			C_S_AXI_DATA_WIDTH => C_S_AXI_DATA_WIDTH,
			C_S_AXI_ADDR_WIDTH => C_S_AXI_ADDR_WIDTH
		)
		port map (
			aclk    => aclk,
			aresetn => aresetn,

			s_axis_tdata  => s_axis_tdata,
			s_axis_tkeep  => s_axis_tkeep,
			s_axis_tvalid => s_axis_tvalid,
			s_axis_tready => s_axis_tready,
			s_axis_tlast  => s_axis_tlast,

			m_axis_tdata  => m_axis_tdata,
			m_axis_tkeep  => m_axis_tkeep,
			m_axis_tvalid => m_axis_tvalid,
			m_axis_tready => m_axis_tready,
			m_axis_tlast  => m_axis_tlast,

			s_axi_awaddr  => s_axi_awaddr,
			s_axi_awprot  => s_axi_awprot,
			s_axi_awvalid => s_axi_awvalid,
			s_axi_awready => s_axi_awready,

			s_axi_wdata   => s_axi_wdata,
			s_axi_wstrb   => s_axi_wstrb,
			s_axi_wvalid  => s_axi_wvalid,
			s_axi_wready  => s_axi_wready,

			s_axi_bresp   => s_axi_bresp,
			s_axi_bvalid  => s_axi_bvalid,
			s_axi_bready  => s_axi_bready,

			s_axi_araddr  => s_axi_araddr,
			s_axi_arprot  => s_axi_arprot,
			s_axi_arvalid => s_axi_arvalid,
			s_axi_arready => s_axi_arready,

			s_axi_rdata   => s_axi_rdata,
			s_axi_rresp   => s_axi_rresp,
			s_axi_rvalid  => s_axi_rvalid,
			s_axi_rready  => s_axi_rready
		);

end architecture rtl;