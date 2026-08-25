library ieee;
use ieee.std_logic_1164.all;

-- AXI4 AxCACHE / AxPROT override shim.
--
-- Combinational pass-through that forces coherent memory attributes onto AXI
-- traffic headed for S_AXI_HPC0_FPD. The AXI DMA ties ARCACHE/AWCACHE to 0, and
-- the SmartConnect preserves them, so transactions reach the CCI marked
-- non-cacheable and are NOT snooped. This shim overrides only:
--   ARCACHE = AWCACHE = "1111"  (write-back read+write-allocate; AxCACHE[3:2]/=00
--                                makes the CCI treat the access as outer-shareable
--                                and snoop the A53 caches)
--   ARPROT  = AWPROT  = "010"   (data, NON-secure, unprivileged -- matches Linux
--                                EL1-NS so the snoop is in the right security
--                                domain; the AMD example uses secure only because
--                                it is bare-metal EL3)
-- Everything else is wired straight through. No clock/reset: there is no state.
--
-- Placement: SmartConnect master output -> S_AXI (this shim) ; M_AXI -> HPC0.
-- Widths default to the HPC0 interface (data 128, addr 49, id 6, user 1).

entity axi_axcache_override is
	generic (
		C_ADDR_WIDTH   : natural := 49;
		C_DATA_WIDTH   : natural := 128;
		C_ID_WIDTH     : natural := 6;
		C_AWUSER_WIDTH : natural := 1;
		C_ARUSER_WIDTH : natural := 1
	);
	port (
		aclk : in std_logic;          -- association only; unused in RTL
		------------------------------------------------------------------
		-- Slave side  (connect to the SmartConnect master output)
		------------------------------------------------------------------
		-- Write address
		s_axi_awid    : in  std_logic_vector(C_ID_WIDTH-1 downto 0);
		s_axi_awaddr  : in  std_logic_vector(C_ADDR_WIDTH-1 downto 0);
		s_axi_awlen   : in  std_logic_vector(7 downto 0);
		s_axi_awsize  : in  std_logic_vector(2 downto 0);
		s_axi_awburst : in  std_logic_vector(1 downto 0);
		s_axi_awlock  : in  std_logic;
		s_axi_awcache : in  std_logic_vector(3 downto 0);
		s_axi_awprot  : in  std_logic_vector(2 downto 0);
		s_axi_awqos   : in  std_logic_vector(3 downto 0);
		s_axi_awuser  : in  std_logic_vector(C_AWUSER_WIDTH-1 downto 0);
		s_axi_awvalid : in  std_logic;
		s_axi_awready : out std_logic;
		-- Write data
		s_axi_wdata   : in  std_logic_vector(C_DATA_WIDTH-1 downto 0);
		s_axi_wstrb   : in  std_logic_vector(C_DATA_WIDTH/8-1 downto 0);
		s_axi_wlast   : in  std_logic;
		s_axi_wvalid  : in  std_logic;
		s_axi_wready  : out std_logic;
		-- Write response
		s_axi_bid     : out std_logic_vector(C_ID_WIDTH-1 downto 0);
		s_axi_bresp   : out std_logic_vector(1 downto 0);
		s_axi_bvalid  : out std_logic;
		s_axi_bready  : in  std_logic;
		-- Read address
		s_axi_arid    : in  std_logic_vector(C_ID_WIDTH-1 downto 0);
		s_axi_araddr  : in  std_logic_vector(C_ADDR_WIDTH-1 downto 0);
		s_axi_arlen   : in  std_logic_vector(7 downto 0);
		s_axi_arsize  : in  std_logic_vector(2 downto 0);
		s_axi_arburst : in  std_logic_vector(1 downto 0);
		s_axi_arlock  : in  std_logic;
		s_axi_arcache : in  std_logic_vector(3 downto 0);
		s_axi_arprot  : in  std_logic_vector(2 downto 0);
		s_axi_arqos   : in  std_logic_vector(3 downto 0);
		s_axi_aruser  : in  std_logic_vector(C_ARUSER_WIDTH-1 downto 0);
		s_axi_arvalid : in  std_logic;
		s_axi_arready : out std_logic;
		-- Read data
		s_axi_rid     : out std_logic_vector(C_ID_WIDTH-1 downto 0);
		s_axi_rdata   : out std_logic_vector(C_DATA_WIDTH-1 downto 0);
		s_axi_rresp   : out std_logic_vector(1 downto 0);
		s_axi_rlast   : out std_logic;
		s_axi_rvalid  : out std_logic;
		s_axi_rready  : in  std_logic;

		------------------------------------------------------------------
		-- Master side  (connect to S_AXI_HPC0_FPD)
		------------------------------------------------------------------
		-- Write address
		m_axi_awid    : out std_logic_vector(C_ID_WIDTH-1 downto 0);
		m_axi_awaddr  : out std_logic_vector(C_ADDR_WIDTH-1 downto 0);
		m_axi_awlen   : out std_logic_vector(7 downto 0);
		m_axi_awsize  : out std_logic_vector(2 downto 0);
		m_axi_awburst : out std_logic_vector(1 downto 0);
		m_axi_awlock  : out std_logic;
		m_axi_awcache : out std_logic_vector(3 downto 0);
		m_axi_awprot  : out std_logic_vector(2 downto 0);
		m_axi_awqos   : out std_logic_vector(3 downto 0);
		m_axi_awuser  : out std_logic_vector(C_AWUSER_WIDTH-1 downto 0);
		m_axi_awvalid : out std_logic;
		m_axi_awready : in  std_logic;
		-- Write data
		m_axi_wdata   : out std_logic_vector(C_DATA_WIDTH-1 downto 0);
		m_axi_wstrb   : out std_logic_vector(C_DATA_WIDTH/8-1 downto 0);
		m_axi_wlast   : out std_logic;
		m_axi_wvalid  : out std_logic;
		m_axi_wready  : in  std_logic;
		-- Write response
		m_axi_bid     : in  std_logic_vector(C_ID_WIDTH-1 downto 0);
		m_axi_bresp   : in  std_logic_vector(1 downto 0);
		m_axi_bvalid  : in  std_logic;
		m_axi_bready  : out std_logic;
		-- Read address
		m_axi_arid    : out std_logic_vector(C_ID_WIDTH-1 downto 0);
		m_axi_araddr  : out std_logic_vector(C_ADDR_WIDTH-1 downto 0);
		m_axi_arlen   : out std_logic_vector(7 downto 0);
		m_axi_arsize  : out std_logic_vector(2 downto 0);
		m_axi_arburst : out std_logic_vector(1 downto 0);
		m_axi_arlock  : out std_logic;
		m_axi_arcache : out std_logic_vector(3 downto 0);
		m_axi_arprot  : out std_logic_vector(2 downto 0);
		m_axi_arqos   : out std_logic_vector(3 downto 0);
		m_axi_aruser  : out std_logic_vector(C_ARUSER_WIDTH-1 downto 0);
		m_axi_arvalid : out std_logic;
		m_axi_arready : in  std_logic;
		-- Read data
		m_axi_rid     : in  std_logic_vector(C_ID_WIDTH-1 downto 0);
		m_axi_rdata   : in  std_logic_vector(C_DATA_WIDTH-1 downto 0);
		m_axi_rresp   : in  std_logic_vector(1 downto 0);
		m_axi_rlast   : in  std_logic;
		m_axi_rvalid  : in  std_logic;
		m_axi_rready  : out std_logic
	);
end entity;

architecture rtl of axi_axcache_override is
	attribute X_INTERFACE_INFO      : string;
	attribute X_INTERFACE_PARAMETER : string;
	attribute X_INTERFACE_INFO of aclk : signal is
		"xilinx.com:signal:clock:1.0 aclk CLK";
	attribute X_INTERFACE_PARAMETER of aclk : signal is
		"ASSOCIATED_BUSIF S_AXI:M_AXI";
begin

	-- Write address: pass through, override cache + prot
	m_axi_awid    <= s_axi_awid;
	m_axi_awaddr  <= s_axi_awaddr;
	m_axi_awlen   <= s_axi_awlen;
	m_axi_awsize  <= s_axi_awsize;
	m_axi_awburst <= s_axi_awburst;
	m_axi_awlock  <= s_axi_awlock;
	m_axi_awcache <= "1111";          -- OVERRIDE: cacheable, write-back R+W allocate
	m_axi_awprot  <= "010";           -- OVERRIDE: data, non-secure, unprivileged
	m_axi_awqos   <= s_axi_awqos;
	m_axi_awuser  <= s_axi_awuser;
	m_axi_awvalid <= s_axi_awvalid;
	s_axi_awready <= m_axi_awready;

	-- Write data
	m_axi_wdata   <= s_axi_wdata;
	m_axi_wstrb   <= s_axi_wstrb;
	m_axi_wlast   <= s_axi_wlast;
	m_axi_wvalid  <= s_axi_wvalid;
	s_axi_wready  <= m_axi_wready;

	-- Write response
	s_axi_bid     <= m_axi_bid;
	s_axi_bresp   <= m_axi_bresp;
	s_axi_bvalid  <= m_axi_bvalid;
	m_axi_bready  <= s_axi_bready;

	-- Read address: pass through, override cache + prot
	m_axi_arid    <= s_axi_arid;
	m_axi_araddr  <= s_axi_araddr;
	m_axi_arlen   <= s_axi_arlen;
	m_axi_arsize  <= s_axi_arsize;
	m_axi_arburst <= s_axi_arburst;
	m_axi_arlock  <= s_axi_arlock;
	m_axi_arcache <= "1111";          -- OVERRIDE
	m_axi_arprot  <= "010";           -- OVERRIDE
	m_axi_arqos   <= s_axi_arqos;
	m_axi_aruser  <= s_axi_aruser;
	m_axi_arvalid <= s_axi_arvalid;
	s_axi_arready <= m_axi_arready;

	-- Read data
	s_axi_rid     <= m_axi_rid;
	s_axi_rdata   <= m_axi_rdata;
	s_axi_rresp   <= m_axi_rresp;
	s_axi_rlast   <= m_axi_rlast;
	s_axi_rvalid  <= m_axi_rvalid;
	m_axi_rready  <= s_axi_rready;

end architecture;