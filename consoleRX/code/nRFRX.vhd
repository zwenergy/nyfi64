library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.math_real.ALL;


entity nRFRX is
  generic ( ackpayloadBytes : integer := 1;
    maxPacketBits : integer := 16;
    crcLen : std_logic := '0'; -- 0: 1 byte, 1: 2 byte
    power : std_logic_vector( 1 downto 0 ) := "01"; -- 00: -18 dB, 01: -12 dB, 10: -6 dB, 11: 0dB
    rxaddr : std_logic_vector( 39 downto 0 ) := x"C0FFEEBEEF";
    channel : std_logic_vector( 6 downto 0 ) := "1100100";
    dataRate : std_logic_vector( 1 downto 0 ) := "10"; -- 00: 1Mbps, 01: 2Mbps, 10: 250kbps
    clkfreq : unsigned( 16 downto 0 ) := to_unsigned( 100000, 17 ); --in kHz
    spiclkfreq : unsigned( 16 downto 0 ) := to_unsigned( 1000, 17 ); --in kHz
    pollingInt : unsigned( 9 downto 0 ) := to_unsigned( 100, 10 ) --in ms
    );
  port ( 
    ackPayload : in std_logic_vector( ackPayloadBytes * 8 - 1 downto 0 );
    ackValid : in  STD_LOGIC;
    miso : in std_logic;
    irq : in std_logic;
     clk : in  STD_LOGIC;
     rst : in  STD_LOGIC;
    rxData : out std_logic_vector( maxPacketBits - 1 downto 0 );
    rxDataValid : out std_logic;
    -- We need a packet size of 8 bits.
    rxLen : out unsigned( 8 downto 0 );
     sclk : out  STD_LOGIC;
    ss : out std_logic;
    mosi : out std_logic;
    ce : out std_logic;
    initDone : out std_logic
    );
end nRFRX;

architecture Behavioral of nRFRX is
component spimaster is
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
end component spimaster;

constant packetsize : unsigned( 3 downto 0 ) := to_unsigned( 8, 4 );
-- Max. number of waiting cycles (100 ms).
constant maxWaitingCycles : integer := integer( ceil( ( 100.0 / ( 1.0 / real( to_integer( clkfreq ) ) ) ) ) );
constant cycles100ms : integer := integer( ceil( ( 100.0 / ( 1.0 / real( to_integer( clkfreq ) ) ) ) ) );
-- 1 ms for simulating.
--constant cycles100ms : integer := integer( ceil( ( 1.0 / ( 1.0 / real( to_integer( clkfreq ) ) ) ) ) );
constant cycles2ms : integer := integer( ceil( ( 2.0 / ( 1.0 / real( to_integer( clkfreq ) ) ) ) ) );
constant cycles1ms : integer := integer( ceil( ( 1.0 / ( 1.0 / real( to_integer( clkfreq ) ) ) ) ) );
constant cycles500ns : integer := integer( ceil( ( 0.5 / ( 1000.0 / real( to_integer( clkfreq ) ) ) ) ) );
constant cyclesPoll : integer := integer( ceil( ( real( to_integer( pollingInt ) ) / ( 1.0 / real( to_integer( clkfreq ) ) ) ) ) );


-- Commands.
constant writeReg : std_logic_vector( 2 downto 0 ) := "001";
constant readReg : std_logic_vector( 2 downto 0 ) := "000";
constant flushRX : std_logic_vector( 7 downto 0 ) := "11100010";
constant writeAckPayP0 : std_logic_vector( 7  downto 0 ) := "10101000";
constant nop : std_logic_vector( 7 downto 0 ) := "11111111";
constant readRXPayload : std_logic_vector( 7 downto  0 ) := "01100001";
constant readRXPayloadLen : std_logic_vector( 7 downto 0 ) := "01100000";

signal rxData_int : std_logic_vector( maxPacketBits - 1 downto 0 );
signal rxDataValid_int : std_logic;
signal initDone_int : std_logic;
signal ss_int : std_logic;
signal spiTXValid : std_logic;
signal spiTXData : std_logic_vector( to_integer( packetsize ) - 1 downto 0 );
signal spiTXBufferFSM : std_logic_vector( 47 downto 0 ); -- 6 bytes max.
signal spiTXBuffer : std_logic_vector( 47 downto 0 ); -- 6 bytes max.
signal spiTXBufferCnt : unsigned( 2 downto 0 ); -- Sending 1 byte at a time.
signal spiTXBufferStartSend : std_logic;
signal spiTXSending : std_logic;
signal spiTXBufferSendDone : std_logic;
signal spiRXvalid : std_logic;
signal spiRXData : std_logic_vector( to_integer( packetsize ) - 1 downto 0 );
signal spiRXBufferCnt : unsigned( 8 downto 0 );
signal spiTXBufferLen : unsigned( spiTXBufferCnt'length -1 downto 0);
signal spiRXBufferLen : unsigned( spiRXBufferCnt'length - 1 downto 0 );
signal cycleCnt : unsigned( integer( ceil( log2( real( maxWaitingCycles + 1 ) ) ) ) - 1 downto 0 );
signal cycleCntRes : std_logic;
signal cycleCntReached : std_logic;
signal cycleCntComp : unsigned( integer( ceil( log2( real( maxWaitingCycles + 1 ) ) ) ) - 1 downto 0 );
signal irq_int : std_logic;

-- Helper signals.
signal lastRXPayloadLen : unsigned( 8 downto 0 );

-- Status bits.
signal lastStatus : std_logic_vector( 7 downto 0 );
signal recvStatus : std_logic;

-- FSM
type FSM_states is ( waitPowerRstSet, waitPowerRst, setPower, waitSetPower, waitPowerUpSet,
                    waitPowerUp, setFeatreg, waitFeatreg, setEN_AA, waitEN_AA,
                    setDPL, waitDPL, setRXP0, waitRXP0, setRFChannel, waitRFChannel,
                    setRFSetup, waitRFSetup, setFlushRX, waitFlushRX, setRXModeWait, 
                    waitRXModeWait, idle, setAck, waitAck, setReadRXLen, waitReadRXLen,
                    setReadRX_PLD, waitReadRX_PLD, validRXPayload, setRSTIRQ, waitRSTIRQ,
                    waitStateSet, waitStatewait, setFIFOStat, waitFIFOStat, setRXErr, waitRXErr );
signal curState : FSM_states;
signal nextState : FSM_states;
-- Comb.
signal stateAfterWait : FSM_states;
-- Sync.
signal stateAfterWaitSeq : FSM_states;

-- A counter to keep track of the last arrival of a package.
signal lastPckArrival : unsigned( integer( ceil( log2( real( cyclesPoll ) ) ) ) - 1 downto 0 );

signal ackPayload_int : std_logic_vector( ackPayloadBytes * 8 - 1 downto 0 );
signal newAck : std_logic;
signal newAckRst : std_logic;
begin

  rxData <= rxData_int;
  rxDataValid <= rxDataValid_int;
  initDone <= initDone_int;
  
  lastRXPayloadLen <= '0' & unsigned( rxData_int( rxData_int'length - 1 downto rxData_int'length - 8 ) );
  
  
  spi : spimaster
    generic map(
      clkfreq => clkfreq,
      packetsize => packetsize,
      ss_default => '1',
      spiclkfreq => spiclkfreq
   )
    port map(
      clk => clk,
      rst => rst,
      ss_in => ss_int,
      din_valid => spiTXValid,
      din => spiTXData,
      miso => miso,
      dout => spiRXData,
      dvalid => spiRXValid,
      mosi => mosi,
      ss_out => ss,
      sclk => sclk
   );
   
  captureAck : process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      ackPayload_int <= ( others => '0' );
      newAck <= '0';
      irq_int <= '1';
    elsif ( rising_edge( clk ) ) then
      irq_int <= irq;
      if ( ackValid = '1' ) then
        ackPayload_int <= ackPayload;
        newAck <= '1';
     
      elsif ( newAckRst = '1' ) then
        newAck <= '0';
      end if;n
    end if;
  end process;
   
  -- SPI sending.
  spiTXBuffSendingSeq : process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      spiTXBufferCnt <= ( others => '0' );
      spiRXBufferCnt <= ( others => '0' );
      spiTXBuffer <= ( others => '0' );
      spiTXBufferSendDone <= '0';
      spiTXValid <= '0';
      spiTXSending <= '0';
      rxData_int <= ( others => '0' );
      lastStatus <= ( others => '0' );
      recvStatus <= '0';
    elsif ( rising_edge( clk ) ) then
      if ( spiTXBufferStartSend = '1' ) then
        spiTXBufferCnt <= spiTXBufferLen;
        spiRXBufferCnt <= spiRXBufferLen;
        spiTXSending <= '1';
        spiTXValid <= '1';
        spiTXBuffer <= spiTXBufferFSM;
        recvStatus <= '0';
       
      elsif ( spiTXSending = '1' ) then
        spiTXValid <= '0';
        if ( spiRXvalid = '1' ) then
          -- Check if the status byte was already set.
          if ( recvStatus = '0' ) then
            -- Store the status bits.
            lastStatus <= spiRXData;
            recvStatus <= '1';
          end if;
       
          if ( spiTXBufferCnt /= 0 ) then
            spiTXBufferCnt <= spiTXBufferCnt - 1;
            -- Shift it down.
            spiTXBuffer( spiTXBuffer'length - to_integer( packetsize ) - 1 downto 0 ) <= 
            spiTXBuffer( spiTXBuffer'length - 1 downto to_integer( packetsize ) );
            spiTXBuffer( spiTXBuffer'length - 1 downto spiTXBuffer'length - to_integer( packetsize ) ) <= ( others => '0' );
            -- Reset the TX valid.
            spiTXValid <= '1';
          else
            spiTXValid <= '0';
          end if;
        
          if ( spiRXBufferCnt /= 0 ) then
            spiRXBufferCnt <= spiRXBufferCnt - 1;
            -- Capture the output and shift it down.
            rxData_int( rxData_int'length - 1 downto rxData_int'length - to_integer( packetsize ) )<=
              spiRXData;
            rxData_int( rxData_int'length - 1 - to_integer( packetsize ) downto 0 ) <= 
            rxData_int( rxData_int'length - 1 downto to_integer( packetsize ) );
          end if;
        
          if ( spiRXBufferCnt <= 1 and spiTXBufferCnt <= 1 ) then
            spiTXBufferCnt <= ( others => '0' );
            spiTXBufferSendDone <= '1';
            spiTXValid <= '0';
            spiTXSending <= '0';
          end if;
        end if;
       
      -- Not sending.
      else
        spiTXBufferCnt <= ( others => '0' );
        spiRXBufferCnt <= ( others => '0' );
        spiTXBufferSendDone <= '0';
        spiTXValid <= '0';
        spiTXSending <= '0';
      end if;
    end if;
  end process;
   
  -- Always use the lowest bits as SPI TX bits.
  spiTXData <= spiTXBuffer( to_integer( packetsize - 1 ) downto 0 );
   
  -- The SS signal is the TX Sending signal inverted.
  ss_int <= not spiTXSending;
   
  cycleCounter : process( clk, rst )  is
  begin
    if ( rst = '1' ) then
      cycleCnt <= ( others => '0' );
    elsif ( rising_edge( clk ) ) then
      if ( cycleCntRes = '1' ) then
        cycleCnt <= ( others => '0' );
      else
        cycleCnt <= cycleCnt + 1;
      end if;
    end if;
  end process;

  -- Counter compare logic.
  cycleCounterCompareLogic : process( cycleCnt, cycleCntRes, cycleCntComp ) is
  begin
    if ( cycleCntRes = '1' ) then
      cycleCntReached <= '0';
    elsif ( cycleCnt >= cycleCntComp ) then
      cycleCntReached <= '1';
    else 
      cycleCntReached <= '0';
    end if;   
  end process;
   
  -- FSM processes
  updateState : process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      curState <= waitPowerRstSet;
      stateAfterWaitSeq <= waitPowerRstSet;
    elsif rising_edge( clk ) then
      curState <= nextState;
      stateAfterWaitSeq <= stateAfterWait;
    end if;
  end process;

  nextStateLogic : process( curState, cycleCntReached, spiTXBufferSendDone, irq_int, newAck,
                            lastRXPayloadLen, lastStatus, stateAfterWaitSeq, rxData_int,
                            lastPckArrival ) is
  begin
    case curState is
      when waitPowerRstSet =>
        nextState <= waitPowerRst;
        stateAfterWait <= waitPowerRstSet;
      when waitPowerRst =>
        if ( cycleCntReached = '1' ) then
          nextState <= setPower;
        else
          nextState <= waitPowerRst;
       end if;
       stateAfterWait <= waitPowerRst;
       
      when setPower =>
        nextState <= waitSetPower;
        stateAfterWait <= setPower;
       
      when waitSetPower =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= waitPowerUpSet;
        else
          nextState <= waitSetPower;
          stateAfterWait <= waitSetPower;
        end if;
       
      when waitPowerUpSet =>
        nextState <= waitPowerUp;
        stateAfterWait <= waitPowerUpSet;
       
      when waitPowerUp =>
        if ( cycleCntReached = '1' ) then
          nextState <= setFeatreg;
        else
          nextState <= waitPowerUp;
        end if;
       
        stateAfterWait <= waitPowerUp;
       
      when setFeatreg =>
        nextState <= waitFeatreg;
        stateAfterWait <= setFeatreg;
       
      when waitFeatreg =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= setEN_AA;
        else
          nextState <= waitFeatreg;
          stateAfterWait <= waitFeatreg;
        end if;
       
      when setEN_AA =>
        nextState <= waitEN_AA;
        stateAfterWait <= setEN_AA;
       
      when waitEN_AA =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= setDPL;
        else
          nextState <= waitEN_AA;
          stateAfterWait <= waitEN_AA;
        end if;
       
      when setDPL =>
        nextState <= waitDPL;
        stateAfterWait <= setDPL;
       
      when waitDPL =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= setRXP0;
        else
          nextState <= waitDPL;
          stateAfterWait <= waitDPL;
        end if;
       
      when setRXP0 =>
        nextState <= waitRXP0;
        stateAfterWait <= setRXP0;
       
      when waitRXP0 =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= setRFChannel;
        else
          nextState <= waitRXP0;
          stateAfterWait <= waitRXP0;
        end if;
       
      when setRFChannel =>
        nextState <= waitRFChannel;
        stateAfterWait <= setRFChannel;
       
      when waitRFChannel =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= setRFSetup;
        else
          nextState <= waitRFChannel;
          stateAfterWait <= waitRFChannel;
        end if;
       
      when setRFSetup =>
        nextState <= waitRFSetup;
        stateAfterWait <= setRFSetup;
       
      when waitRFSetup =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= setFlushRX;
        else
          nextState <= waitRFSetup;
          stateAfterWait <= waitRFSetup;
        end if;
       
      when setFlushRX =>
        nextState <= waitFlushRX;
        stateAfterWait <= setFlushRX;
       
      when waitFlushRX =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= setRXModeWait;
        else
          nextState <= waitFlushRX;
          stateAfterWait <= waitFlushRX;
        end if;
       
      when setRXModeWait =>
        nextState <= waitRXModeWait;
        stateAfterWait <= setRXModeWait;
       
      when waitRXModeWait =>
        if ( cycleCntReached = '1' ) then
          nextState <= idle;
        else
          nextState <= waitRXModeWait;
        end if;
        stateAfterWait <= waitRXModeWait;
       
      when idle =>
        -- Either the IRQ went down or we have not received
        -- a package in a while.
        if ( irq_int = '0' or lastPckArrival = 0 ) then
          nextState <= setReadRXLen;
          -- no new data and new ACK.
        elsif ( newAck = '1' ) then
          nextState <= setAck;
        else
          nextState <= idle;
        end if;
       
        stateAfterWait <= idle;
       
      when setAck =>
        nextState <= waitAck;
        stateAfterWait <= setAck;
       
      when waitAck =>
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= idle;
        else
          nextState <= waitAck;
          stateAfterWait <= waitAck;
        end if;
       
      when setReadRXLen =>
        nextState <= waitReadRXLen;
        stateAfterWait <= setReadRXLen;
       
      when waitReadRXLen =>
        -- Check if there is data to receive.
        if ( spiTXBufferSendDone = '1' ) then 
          -- If a received packet is 0,
          -- we omit it and flush the RX buffer.
          if ( lastRXPayloadLen = to_unsigned( 0, lastRXPayloadLen'length ) ) then
               stateAfterWait <= setRxErr;
            nextState <= waitStateSet;
          else
            stateAfterWait <= setReadRX_PLD;
            nextState <= waitStateSet;
          end if;
        else
          -- Not done with sending yet.
          nextState <= waitReadRXLen;
          stateAfterWait <= waitReadRXLen;
        end if;
       
      when setReadRX_PLD =>
        nextState <= waitReadRX_PLD;
        stateAfterWait <= setReadRX_PLD;
       
      when waitReadRX_PLD =>
        if ( spiTXBufferSendDone = '1' ) then 
          nextState <= waitStateSet;
          stateAfterWait <= validRXPayload;
        else
          nextState <= waitReadRX_PLD;
          stateAfterWait <= waitReadRX_PLD;
        end if; 
       
      when validRXPayload =>
        nextState <= setRSTIRQ;
        stateAfterWait <= validRXPayload;
       
      when setRSTIRQ =>
        nextState <= waitRSTIRQ;
        stateAfterWait <= setRSTIRQ;
       
      when waitRSTIRQ =>
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= setFIFOStat;
        else
          nextState <= waitRSTIRQ;
          stateAfterWait <= waitRSTIRQ;
        end if;
       
      when waitStateSet =>
        nextState <= waitStatewait;
        stateAfterWait <= stateAfterWaitSeq;
       
      when waitStatewait =>
        if ( cycleCntReached = '1' ) then
          nextState <= stateAfterWaitSeq;
          stateAfterWait <= waitStatewait;
        else
          nextState <= waitStatewait;
          stateAfterWait <= stateAfterWaitSeq;
        end if;
       
      when setFIFOStat => 
        nextState <= waitFIFOStat;
        stateAfterWait <= setFIFOStat;
       
      when waitFIFOStat =>
        if ( spiTXBufferSendDone = '1' ) then
          -- Check if there is data left to read.
          if ( rxData_int( rxData_int'length - 8 ) = '1' ) then
            nextState <= waitStateSet;
            stateAfterWait <= idle;
          else 
            nextState <= waitStateSet;
            stateAfterWait <= setReadRXLen;
          end if;       
         
        else
          nextState <= waitFIFOStat;
          stateAfterWait <= waitFIFOStat;
        end if;
          
      when setRXErr =>
        nextState <= waitRXErr;
        stateAfterWait <= setRXErr;
       
      when waitRXErr =>
        -- Check if sending is done.
        if ( spiTXBufferSendDone = '1' ) then
          nextState <= waitStateSet;
          stateAfterWait <= idle;
        else
          nextState <= waitRXErr;
          stateAfterWait <= waitRXErr;
        end if;
             
      when others =>
        -- dummy.
        nextState <= setPower;
        stateAfterWait <= setPower;
    end case;
  end process;
   
  stateOutputLogic : process( curState, ackPayload_int, lastRXPayloadLen, stateAfterWaitSeq ) is
  begin
    case curState is
      when waitPowerRstSet =>
        cycleCntRes <= '1';
        cycleCntComp <= to_unsigned( cycles100ms, cycleCntComp'length );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when waitPowerRst => 
        cycleCntRes <= '0';
        cycleCntComp <= to_unsigned( cycles100ms, cycleCntComp'length );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setPower =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        -- We will send 2 bytes.
        spiTXBufferLen <= to_unsigned( 2, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- Set power up, RX mode and crc (register 0).
        spiTXBufferFSM( 15 downto 0 ) <= "00001" & crcLen & "11" & writeReg & "00000";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 16 ) <= ( others => '0' );
        newAckRst <= '0';
       
      when waitSetPower =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when waitPowerUpSet =>
        cycleCntRes <= '1';
        cycleCntComp <= to_unsigned( cycles2ms, cycleCntComp'length );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when waitPowerUp => 
        cycleCntRes <= '0';
        cycleCntComp <= to_unsigned( cycles2ms, cycleCntComp'length );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setFeatreg =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        -- We will send 2 bytes.
        spiTXBufferLen <= to_unsigned( 2, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- Set feature register 1D (DPL, ACK_PAY)
        spiTXBufferFSM( 15 downto 0 ) <= "00000" & "110" & writeReg & "11101";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 16 ) <= ( others => '0' );
        newAckRst <= '0';
       
      when waitFeatReg => 
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setEN_AA =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        -- We will send 2 bytes.
        spiTXBufferLen <= to_unsigned( 2, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- Set enhanced shockburst reg 01 (EN_AA)
        spiTXBufferFSM( 15 downto 0 ) <= "00000001" & writeReg & "00001";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 16 ) <= ( others => '0' );
        newAckRst <= '0';
       
      when waitEN_AA => 
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setDPL => 
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        -- We will send 2 bytes.
        spiTXBufferLen <= to_unsigned( 2, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- Set DYNDPD reg 1C
        spiTXBufferFSM( 15 downto 0 ) <= "00000001" & writeReg & "11100";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 16 ) <= ( others => '0' );
        newAckRst <= '0';
       
      when waitDPL =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setRXP0 =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        -- We will send 6 bytes.
        spiTXBufferLen <= to_unsigned( 6, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- SetRXADDRP0 reg 0A
        spiTXBufferFSM( 47 downto 0 ) <= rxaddr & writeReg & "01010";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 48 ) <= ( others => '0' );
        newAckRst <= '0';
       
      when waitRXP0 =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setRFChannel =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        -- We will send 2 bytes.
        spiTXBufferLen <= to_unsigned( 2, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- set RF_CH reg 05
        spiTXBufferFSM( 15 downto 0 ) <= '0' & channel & writeReg & "00101";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 16 ) <= ( others => '0' );
        newAckRst <= '0';
       
      when waitRFChannel =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setRFSetup =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        -- We will send 2 bytes.
        spiTXBufferLen <= to_unsigned( 2, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- set RF_Setup reg 06
        spiTXBufferFSM( 15 downto 0 ) <= "00" & dataRate(1) & '0' & dataRate(0) & power & '0' & writeReg & "00110";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 16 ) <= ( others => '0' );
        newAckRst <= '0';
       
      when waitRFSetup =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setFlushRX =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        -- We will send 1 byte.
        spiTXBufferLen <= to_unsigned( 1, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- set RF_Setup reg 06
        spiTXBufferFSM( 7 downto 0 ) <= flushRX;
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 8 )  <= ( others => '0' );
        newAckRst <= '0';
       
      when waitFlushRX =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '0';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setRXModeWait =>
        cycleCntRes <= '1';
        cycleCntComp <= to_unsigned( cycles1ms, cycleCntComp'length );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when waitRXModeWait =>
        cycleCntRes <= '0';
        cycleCntComp <= to_unsigned( cycles1ms, cycleCntComp'length );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when idle =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setAck =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= to_unsigned( ackPayloadBytes + 1, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- Write to ACK Payload buffer
        spiTXBufferFSM( 7 downto 0 ) <= writeAckPayP0;
        spiTXBufferFSM( 7 + ackPayloadBytes * 8 downto 8 ) <= ackPayload_int;
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 8 + ackPayloadBytes * 8 )  <= ( others => '0' );
        newAckRst <= '1';
       
      when waitAck =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setReadRXLen =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        -- Only command
        spiTXBufferLen <= to_unsigned( 1, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        -- Receive status and length
        spiRXBufferLen <= to_unsigned( 2, spiRXBufferLen'length );
        -- Read register 0x11
        spiTXBufferFSM( 7 downto 0 ) <= readRXPayloadLen;
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 8 )  <= ( others => '0' );
        newAckRst <= '0';
       
      when waitReadRXLen =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setReadRX_PLD =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        -- Only command
        spiTXBufferLen <= to_unsigned( 1, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        -- Receive status + the number of bytes to be received.
        spiRXBufferLen <= lastRXPayloadLen( spiRXBufferLen'length - 1 downto 0 ) + 1;
        -- Read register RX payload
        spiTXBufferFSM( 7 downto 0 ) <= readRXPayload;
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 8 )  <= ( others => '0' );
        newAckRst <= '0';
       
      when waitReadRX_PLD =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when validRXPayload =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '1';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= lastRXPayloadLen( spiRXBufferLen'length - 1 downto 0 ) + 1;
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
       
      when setRSTIRQ =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        -- Write command and one byte.
        spiTXBufferLen <= to_unsigned( 2, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        -- Receive status.
        spiRXBufferLen <= to_unsigned( 1, spiRXBufferLen'length );
        -- Write status register (07)
        spiTXBufferFSM( 15 downto 0 ) <= "01101110" & writeReg & "00111";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 16 )  <= ( others => '0' );
        newAckRst <= '0';
       
      when waitRSTIRQ =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0'; 
       
      when waitStatewait | waitStateSet =>
        if ( curState = waitStatewait ) then
          cycleCntRes <= '0';
        else 
          cycleCntRes <= '1';
        end if;

        cycleCntComp <= ( to_unsigned( cycles500ns, cycleCntComp'length ) );
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
        rxDataValid_int <= '0';
        -- For the rest of the signals, we have to check where in the FSM we are.
        case stateAfterWaitSeq is
          when waitPowerRstSet | waitPowerRst | setPower | waitSetPower | waitPowerUpSet | waitPowerUp |
              setFeatreg | waitFeatReg | setEN_AA | waitEN_AA | setDPL | waitDPL | setRXP0 | waitRXP0 |
              setRFChannel | waitRFChannel | setRFSetup | waitRFSetup | setFlushRX | waitFlushRX =>
            ce <= '0';
            initDone_int <= '0';
          
          when setRXModeWait | waitRXModeWait =>
            ce <= '1';
            initDone_int <= '0';
          
          when idle | setAck | waitAck | setReadRXLen | waitReadRXLen | setReadRX_PLD | waitReadRX_PLD | 
              validRXPayload | setRSTIRQ | waitRSTIRQ | setFIFOStat | waitFIFOStat | setRXErr =>
            ce <= '1';
            initDone_int <= '1';
          
          when others => 
            -- This should never be the case.
            ce <= '0';
            initDone_int <= '0';
      end case;
       
      when setFIFOStat =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        -- Read command
        spiTXBufferLen <= to_unsigned( 1, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        -- Receive status and answer reg.
        spiRXBufferLen <= to_unsigned( 2, spiRXBufferLen'length );
        -- Read FIFO status reg (17)
        spiTXBufferFSM( 7 downto 0 ) <= readReg & "10111";
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 8 )  <= ( others => '0' );
        newAckRst <= '0';
      
      when waitFIFOStat =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
          
      when setRXErr =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        -- We will send 1 byte.
        spiTXBufferLen <= to_unsigned( 1, spiTXBufferLen'length );
        spiTXBufferStartSend <= '1';
        spiRXBufferLen <= ( others => '0' );
        -- set flushRX.
        spiTXBufferFSM( 7 downto 0 ) <= flushRX;
        spiTXBufferFSM( spiTXBufferFSM'length - 1 downto 8 )  <= ( others => '0' );
        newAckRst <= '0';
       
      when waitRXErr =>
        cycleCntRes <= '1';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '1';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
        
      when others =>
        --dummy
        cycleCntRes <= '0';
        cycleCntComp <= ( others => '0' );
        initDone_int <= '0';
        rxDataValid_int <= '0';
        ce <= '1';
        spiTXBufferLen <= ( others => '0' );
        spiTXBufferStartSend <= '0';
        spiRXBufferLen <= ( others => '0' );
        spiTXBufferFSM <= ( others => '0' );
        newAckRst <= '0';
    end case;
  end process;
   
  -- Process to update the rxLen
  updateRxLen:process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      rxLen <= ( others => '0' );
    elsif rising_edge( clk ) then
      if ( spiTXBufferStartSend = '1' ) then
        rxLen <= spiRXBufferLen;
      end if;
    end if;
  end process;

  -- A counter which resets when we reset IRQ goes low, 
  -- meaning we received a new package. Somehow, this
  -- seems to get stuck from time to time, that's why
  -- we actively poll if we have not received a new
  -- package after a set amount of cycles.
  pollTimer : process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      lastPckArrival <= to_unsigned( cyclesPoll, lastPckArrival'length );
    elsif ( rising_edge( clk ) ) then
      -- Reset if we are switching to the 
      -- setReadRXLen state (meaning we are
      -- trying to receive a package).
      if ( nextState = setReadRXLen ) then
        lastPckArrival <= to_unsigned( cyclesPoll, lastPckArrival'length );
      else
        if ( lastPckArrival /= 0 ) then
          lastPckArrival <= lastPckArrival - 1 ;
        end if;
      end if;
    end if;
  end process;
  
end Behavioral;

