library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.math_real.ALL;

library ice;

entity n64_top is
  generic( 
    maxPacketBits : integer := 40;
    clkfreq : unsigned( 16 downto 0 ) := to_unsigned( 24000, 17 ); --in kHz
   ackpayloadBytes : integer := 1
  );
  
  port( 
    n64Serial : inout std_logic;
    nrfIRQ : in std_logic;
    miso : in std_logic;
    mosi : out std_logic;
    ss : out std_logic;
    ce : out std_logic;
    rfSCLK : out std_logic;

    -- FRAM SPI signals.
    memMISO : in std_logic;
    memMOSI : out std_logic;
    memSS : out std_logic;
    memSCLK : out std_logic );
end n64_top;

architecture Behavioral of n64_top is
signal rxData : std_logic_vector( maxPacketBits - 1 downto 0 );
signal rxLen : unsigned( 8 downto 0 );
signal rxDataValid : std_logic;
signal rumbleMode : std_logic;
signal rumbleModeDel : std_logic;

-- Low freq. clock.
signal clkL : std_logic;

-- RST counter. We hold reset down for some clk cycles.
signal rstCnt : unsigned( 1 downto 0 ) := ( others => '0' );
signal rst : std_logic := '1';

-- CLK signal
signal clk : std_logic;

-- Currently not used.
signal ackPayload : std_logic_vector( ackPayloadBytes * 8 - 1 downto 0 );
signal ackValid : STD_LOGIC;
signal initDone : std_logic;

-- Lattice CLK
component SB_HFOSC  
GENERIC( CLKHF_DIV :string :="0b01");
PORT(
        CLKHFEN: IN STD_LOGIC ;
        CLKHFPU: IN STD_LOGIC;
        CLKHF:OUT STD_LOGIC
        );
END COMPONENT;

begin

  -- Clock generation.
  -- Divide the 48 MHz clock by two.
  u_osc : SB_HFOSC
    GENERIC MAP(CLKHF_DIV =>"0b01")
    port map(
      CLKHFEN  => '1',
      CLKHFPU  => '1',
      CLKHF     => clk
   );
  clkL <= clk;
  

  n64ent : entity work.n64Comm( Behavioral )
    generic map(
      --32 + 8 status bits from the NRF chip.
      maxPacketBits => maxPacketBits,
      clkfreq => clkfreq )
    
    port map(
      serialDatIn => n64Serial,
      clk => clkL,
      rst => rst,
      rxData => rxData,
      rxLen => rxLen,
      rxDataValid => rxDataValid,
      serialDatOut => n64Serial,
      rumbleMode => rumbleMode,
      
      -- FRAM SPI signals.
      memMISO => memMISO,
      memMOSI => memMOSI,
      memSS => memSS,
      memSCLK => memSCLK     
   );
    
     rstGen:process( clkL ) is
     begin
       if ( rising_edge( clkL ) ) then 
         if ( rstCnt /= "11" ) then
           rstCnt <= rstCnt + 1;
         end if;
       end if;
     end process;
     
    rst <= '1' when ( rstCnt /= "11" ) else
           '0';
    
    -- Delay rumbleMode by one clock.
    process( clkL, rst ) is
    begin
      if ( rst = '1' ) then
        rumbleModeDel <= '0';
      elsif( rising_edge( clkL ) ) then
        rumbleModeDel <= rumbleMode;
      end if;
    end process;
    
    -- rumbleMode is the payload
    ackPayload <= ( 0 => rumbleMode, others => '0' );
    -- we set valid everytime the rumbleMode changes.
    ackValid <= rumbleMode xor rumbleModeDel;
   
   nrf24ent : entity work.nRFRX( Behavioral )
      generic map(
        ackpayloadBytes => 1,
        maxPacketBits => maxPacketBits,
        crcLen => '0', -- 0: 1 byte, 1: 2 byte
        power => "01", -- 00: -18 dB, 01: -12 dB, 10: -6 dB, 11: 0dB
        rxaddr => x"C0FFEEBEEF",
        channel => "1100100",
        dataRate => "10", -- 00: 1Mbps, 01: 2Mbps, 10: 250kbps
        clkfreq => clkfreq, --in kHz
        spiclkfreq => to_unsigned( 1000, 17 ),
        pollingInt => to_unsigned( 100, 10 ) --in kHz
      )
    
      port map(
        ackPayload => ackPayload,
        ackValid => ackValid,
        miso => miso,
        irq => nrfIRQ,
        clk =>clkL,
        rst => rst,
        rxData => rxData,
        rxDataValid => rxDataValid,
        -- We need a packet size of 8 bits.
        rxLen => rxLen,
        sclk => rfSCLK,
        ss => ss,
        mosi => mosi,
        ce => ce,
        initDone => initDone
      );


end Behavioral;

