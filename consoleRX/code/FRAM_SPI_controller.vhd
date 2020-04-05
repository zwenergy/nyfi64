library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.math_real.ALL;
use IEEE.NUMERIC_STD.ALL;


entity FRAM_SPI_controller is
  generic(
      clkfreq : unsigned( 16 downto 0 ) := to_unsigned( 100000, 17 )
    );
  port(
      dataIn : in std_logic_vector( 7 downto 0 );
      writeEn : in std_logic;
      addr : in std_logic_vector( 14 downto 0 );
      validIn : in std_logic;
      dataOut : out std_logic_vector( 7 downto 0 );
      validOut : out std_logic;
      busy : out std_logic;
      
      
      -- SPI signals.
      spiDataIn : in std_logic_vector( 7 downto 0 );
      spiDataInValid : in std_logic;
      spiDataOut : out std_logic_vector( 7 downto 0);
      spiDataOutValid : out std_logic;
      spiSS : out std_logic;
      
      -- General signals.
      clk : in std_logic;
      rst : in std_logic
   );
end FRAM_SPI_controller;

architecture Behavioral of FRAM_SPI_controller is
-- We control most using one FSM.
type FSM_States is ( idle, sendWREN, sendWRITE, sendAddrMSB, sendAddrLSB,
                     sendData, sendREAD, rxByte, rxDone, waitState, wait1us );
signal curState : FSM_States;
signal nextState : FSM_States;
signal stateAfterWait : FSM_States;
signal stateAfterWaitSeq : FSM_States;
-- A signal used to show whether we are currently sending or receiving.
signal txAct : std_logic;

-- Sync. signals of the ingoing signals.
signal dataInSeq : std_logic_vector( 7 downto 0 );
signal addrSeq : std_logic_vector( 14 downto 0 );
signal dataOutInt : std_logic_vector( 7 downto 0 );

-- FRAM SPI commands.
constant cmdWREN : std_logic_vector( 7 downto 0 ) := "00000110";
constant cmdREAD : std_logic_vector( 7 downto 0 ) := "00000011";
constant cmdWRITE : std_logic_vector( 7 downto 0 ) := "00000010";

-- More constants.
constant cycles1us : integer := integer( ceil( 1.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) );
constant bits1us : integer := integer( ceil( log2( 1.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );

-- A counter to wait 1us.
signal cnt : unsigned( bits1us - 1  downto 0 );

begin
   
   dataOut <= dataOutInt;
   
   -- Capture input.
   process( clk, rst ) is
   begin
   
      if ( rst = '1' ) then
         dataInSeq <= ( others => '0' );
         addrSeq <= ( others => '0' );
         dataOutInt <= ( others => '0' );
      elsif ( rising_edge( clk ) ) then
         if ( validIn = '1' ) then
            dataInSeq <= dataIn;
            addrSeq <= addr;
         end if;
         
         if ( spiDataInValid = '1' ) then
            dataOutInt <= spiDataIn;
         end if;
         
         stateAfterWaitSeq <= stateAfterWait;
      end if;
   end process;
   
   
   -- Switch FSM state process.
   process( clk, rst ) is
   begin
      if ( rst = '1' ) then
         curState <= idle;
      elsif( rising_edge( clk ) ) then
         curState <= nextState;
      end if;
   end process;
   
   -- Determine next state.
   process( validIn, spiDataInValid, curState, writeEn, stateAfterWaitSeq, txAct,
            cnt ) is
   begin
      case curState is
         when idle =>
            if ( validIn = '1' ) then
               if ( writeEn = '1' ) then
                  nextState <= sendWREN;
               else
                  nextState <= sendREAD;
               end if;
            else
               nextState <= idle;
            end if;
            
            stateAfterWait <= idle;
         
         when sendWREN =>
            nextState <= waitState;
            stateAfterWait <= wait1us;
            
         when wait1us =>
            if ( cnt = 0 ) then
               nextState <= sendWRITE;
            else
               nextState <= wait1us;
            end if;
            stateAfterWait <= wait1us;
            
         when waitState =>
            if ( spiDataInValid = '1' ) then
               nextState <= stateAfterWaitSeq;
            else
               nextState <= waitState;
            end if;
            stateAfterWait <= stateAfterWaitSeq;
            
         when sendWRITE =>
            nextState <= waitState;
            stateAfterWait <= sendAddrMSB;
            
         when sendAddrMSB =>
            nextState <= waitState;
            stateAfterWait <= sendAddrLSB;
            
         when sendAddrLSB =>
            nextState <= waitState;
            if ( txAct = '1' ) then
               stateAfterWait <= sendData;
            else
               stateAfterWait <= rxByte;
            end if;
            
         when sendData =>
            nextState <= waitState;
            stateAfterWait <= idle;
            
         when sendREAD =>
           nextState <= waitState;
           stateAfterWait <= sendAddrMSB;
           
         when rxByte =>
            if ( spiDataInValid = '1' ) then
               nextState <= rxDone;
            else
               nextState <= rxByte;
            end if;
            
            stateAfterWait <= rxByte;
            
         when rxDone =>
            nextState <= idle;
            stateAfterWait <= rxDone;
            
         when others =>
            nextState <= idle;
            stateAfterWait <= idle;
       
      end case;
   end process;
   
   -- Set the txAct signal, indicating if we are currently receiving or
   -- transmitting.
   process( clk, rst ) is
   begin
      if ( rst = '1' ) then
         txAct <= '0';
      elsif ( rising_edge( clk ) ) then
         if ( curState = sendWRITE ) then
            txAct <= '1';
         elsif ( curState = sendREAD ) then
            txAct <= '0';
         end if;
      end if;
   end process;
   
   -- And the process to control the FSM's output signals.
   process( curState, addrSeq, dataInSeq ) is
   begin
      case curState is
         when idle =>
            spiDataOut <= ( others => '0' );
            spiDataOutValid <= '0';
            spiSS <= '1';
            validOut <= '0';
            busy <= '0';
            
         when sendWREN =>
            spiDataOut <= cmdWREN;
            spiDataOutValid <= '1';
            spiSS <= '1';
            validOut <= '0';
            busy <= '1';
            
         when waitState =>
            spiDataOut <= ( others => '0' );
            spiDataOutValid <= '0';
            spiSS <= '0';
            validOut <= '0';
            busy <= '1';
            
         when sendWRITE =>
            spiDataOut <= cmdWRITE;
            spiDataOutValid <= '1';
            spiSS <= '1';
            validOut <= '0';
            busy <= '1';
            
         when sendAddrMSB =>
            spiDataOut <= "0" & addrSeq( 14 downto 8 );
            spiDataOutValid <= '1';
            spiSS <= '0';
            validOut <= '0';
            busy <= '1';
            
         when sendAddrLSB =>
            spiDataOut <= addrSeq( 7 downto 0 );
            spiDataOutValid <= '1';
            spiSS <= '0';
            validOut <= '0';
            busy <= '1';
            
         when sendData =>
            spiDataOut <= dataInSeq;
            spiDataOutValid <= '1';
            spiSS <= '0';
            validOut <= '0';
            busy <= '1';
            
         when sendREAD =>
            spiDataOut <= cmdREAD;
            spiDataOutValid <= '1';
            spiSS <= '0';
            validOut <= '0';
            busy <= '1';
            
         when rxByte =>
            spiDataOut <= ( others => '0' );
            spiDataOutValid <= '0';
            spiSS <= '0';
            validOut <= '0';
            busy <= '1';
            
            
         when rxDone =>
            spiDataOut <= ( others => '0' );
            spiDataOutValid <= '0';
            spiSS <= '1';
            validOut <= '1';
            busy <= '1';
            
         when wait1us =>
            spiDataOut <= ( others => '0' );
            spiDataOutValid <= '0';
            spiSS <= '1';
            validOut <= '0';
            busy <= '1';
            
         when others =>
            spiDataOut <= ( others => '0' );
            spiDataOutValid <= '0';
            spiSS <= '1';
            validOut <= '0';
            busy <= '1';            
            
      end case;
   end process;
   
   -- A counter which starts counts down from 1us to 0
   -- whenever we enter the "wait1us" state.
   process( clk, rst ) is
   begin
      if ( rst = '1' ) then
         cnt <= to_unsigned( cycles1us, cnt'length );
      elsif ( rising_edge( clk ) ) then
         if ( curState = wait1us and cnt /= 0 ) then
            cnt <= cnt - 1;
            
         else
            cnt <=  to_unsigned( cycles1us, cnt'length );
         end if;
      end if;
   end process;


end Behavioral;

