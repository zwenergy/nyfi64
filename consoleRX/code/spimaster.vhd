library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.math_real.ALL;
use IEEE.NUMERIC_STD.ALL;


entity spimaster is
  generic(
    clkfreq : unsigned( 16 downto 0 ) := to_unsigned( 100000, 17 ); --in kHz
    spiclkfreq : unsigned( 16 downto 0 ) := to_unsigned( 1000, 17 ); --in kHz
    packetsize : unsigned( 3 downto 0 ) := to_unsigned( 8, 4 ); -- in bit
    ss_default : std_logic := '1'
    );
  port(
    clk : in std_logic;
    rst : in std_logic;
    ss_in : in std_logic;
    din_valid : in std_logic;
    din : in std_logic_vector( to_integer( packetsize ) - 1 downto 0 );
    miso : in std_logic;
    dout : out std_logic_vector( to_integer( packetsize ) - 1 downto 0 );
    dvalid : out std_logic;
    mosi : out std_logic;
    ss_out : out std_logic;
    sclk : out std_logic
    );
    
end spimaster;

architecture Behavioral of spimaster is
component clkgen is
  generic(
    clkfreqin : unsigned( 16 downto 0 ) := to_unsigned( 100000, 17 ); --in kHz
    clkfreqout : unsigned( 16 downto 0 ) := to_unsigned( 1000, 17 ); --in kHz
    rst_val : std_logic := '0'
    );
  port (
    clk_in : in std_logic;
    rst : in std_logic;
    clk_out : out std_logic
    );
end component;

signal rxbuf : std_logic_vector( to_integer( packetsize ) - 1 downto 0 );
signal bitcnt : unsigned( integer( ceil( log2( real( to_integer( packetsize + 1 ) ) ) ) ) - 1 downto 0 );
signal spiclk : std_logic;
signal spiclkprev : std_logic;
signal ssprev : std_logic;
signal spiclkrise : std_logic;
signal spiclkfall : std_logic;
signal ssfall : std_logic;
signal miso_int : std_logic;
signal ss_int : std_logic;
signal din_int : std_logic_vector( to_integer( packetsize ) - 1 downto 0 );
signal clkgen_res : std_logic;
begin

  spiclkgen : clkgen
  generic map(
    clkfreqin => clkfreq,
    clkfreqout => spiclkfreq,
    rst_val => '0' )
  port map(
    clk_in => clk,
    rst => clkgen_res,
    clk_out => spiclk );

  sampleinputs : process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      ss_int <= '1';
      miso_int <= '0';
    elsif ( rising_edge( clk ) ) then
      ss_int <= ss_in;
      miso_int <= miso;
    end if;
  end process;
  
  innersignals:process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      rxbuf <= ( others => '0' );
      bitcnt <= ( others => '0' );
      spiclkprev <= '0';
      din_int <= ( others => '0' );
      ssprev <= '0';
    elsif ( rising_edge( clk ) ) then
      spiclkprev <= spiclk;
      ssprev <= ss_int;
      
      if ( din_valid  ='1' ) then
        din_int <= din;
      elsif ( spiclkfall = '1' ) then
        din_int <= ( din_int( din_int'length - 2 downto 0 ) & '0' );
      end if;
      
      if ( ssfall = '1' ) then
        bitcnt <= ( others => '0' );
      elsif ( spiclkfall = '1' ) then
        bitcnt <= bitcnt + 1;
      elsif ( bitcnt = packetsize ) then
        bitcnt <= ( others => '0' );
      end if;
      
      -- rx buffer.
      if ( spiclkrise = '1' ) then
        rxbuf <= ( rxbuf( rxbuf'length - 2 downto 0 ) & miso_int );
      end if;
      
    end if;
  end process;

  spiclkrise <= spiclk and not spiclkprev;
  spiclkfall <= not spiclk and spiclkprev;
  ssfall <= not ss_int and ssprev;
  sclk <= spiclk;
  dout <= rxbuf;
  dvalid <= '1' when ( bitcnt = packetsize ) else '0';
  
  ss_out <= ss_int;
  mosi <= din_int( din_int'length - 1 );
  
  clkgen_res <= rst or ss_in;

end Behavioral;

