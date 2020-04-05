library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.math_real.ALL;


entity n64Comm is
    generic ( 
             maxPacketBits : integer := 40;
             clkfreq : unsigned( 16 downto 0 ) := to_unsigned( 100000, 17 ) --in kHz
             );
    port ( serialDatIn : in  std_logic;
           clk : in  std_logic;
           rst : in  std_logic;
           rxData : in std_logic_vector( maxPacketBits - 1 downto 0 );
           rxLen : in unsigned( 8 downto 0 );
           rxDataValid : in std_logic;
           serialDatOut : out std_logic;
           rumbleMode : out std_logic;
           
           -- FRAM SPI signals.
           memMISO : in std_logic;
           memMOSI : out std_logic;
           memSS : out std_logic;
           memSCLK : out std_logic           
        );
end n64Comm;

architecture Behavioral of n64Comm is
constant cycles2us : integer := integer( ceil( ( 2.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant bits2us : integer := integer( ceil( log2( 2.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant cycles5us : integer := integer( ceil( ( 5.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant bits5us : integer := integer( ceil( log2( 5.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant cycles1us : integer := integer( ceil( ( 1.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant cycles3us : integer := integer( ceil( ( 3.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant cycles4us : integer := integer( ceil( ( 4.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant bits4us : integer := integer( ceil( log2( 4.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant cycles1050us : integer := integer( ceil( ( 1050.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant bits1050us : integer := integer( ceil( log2( 1050.0 / ( 1.0 / real( to_integer( clkfreq ) / 1000 ) ) ) ) );
constant noPakStatus : std_logic_vector( 23 downto 0 ) := x"050002";
constant pakStatus : std_logic_vector( 23 downto 0 ) := x"050001";
constant rumbleAddrStart : std_logic_vector( 10 downto 0 ) := "10000000000";

-- N64 Commands.
constant cmdReadButtons : std_logic_vector( 7 downto 0 ) := x"01";
constant cmdReadStatus : std_logic_vector( 7 downto 0 ) := x"00";
constant cmdReadMem : std_logic_vector( 7 downto 0 ) := x"02";
constant cmdWriteMem : std_logic_vector( 7 downto 0 ) := x"03";

signal serSyn : std_logic;
signal serSynDel : std_logic;
signal serFall : std_logic;
signal serValue : std_logic;
signal serValueValid : std_logic;
-- We need to countdown 2 us.
signal bitTimeCnt : unsigned( bits2us - 1 downto 0 );
signal bitTimeCntReached : std_logic;
signal bitTimeCntRst : std_logic;
signal bitTimeCntZero : std_logic;
signal bitTimeCntZeroDel : std_logic;
--Memory to hold the last pressed buttons.
signal lastButtons : std_logic_vector( 31 downto 0 );
signal lastStatus : std_logic_vector( 23 downto 0 );
-- Temp. buffers for RX.
signal lastButtonsRXBuffer : std_logic_vector( lastButtons'length - 1 downto 0 );
signal lastStatusRXBuffer : std_logic_vector( lastStatus'length -1 downto 0 );
-- A packet counter (to max 32).
signal packetCnt : unsigned( 5 downto 0 );
signal packetCntRst : std_logic;
signal packetCntDel : unsigned( 5 downto 0 );
-- A timeout counter (max. time for receiving 32 packets, 1 packet = 32us, ~1050us).
signal timeoutCnt : unsigned( bits1050us - 1 downto 0 );
signal timeoutCntRst : std_logic;
signal timeoutCntReached : std_logic;

-- Signal for writing/reading.
signal lastAddr : std_logic_vector( 10 downto 0 );

-- writeMode: 0 -> Rumble region, 1 -> memory region.
signal writeMode : std_logic;
-- pakStatus: 0 -> rumble, 1-> mem pak.
signal curPakStatus : std_logic;
signal rumbleLatch : std_logic_vector( 7 downto 0 );
signal rumbleOn : std_logic;
signal crc8Result : std_logic_vector( 7 downto 0 );
signal crc8En : std_logic;
signal crc8Rst : std_logic;
signal crcDatIn : std_logic_vector( 7 downto 0 );

-- Shift register for packetized RX N64 command.
signal n64RXPacket: std_logic_vector( 7 downto 0 );
signal packetTimeCnt : unsigned( bits5us -1 downto 0 );
signal packetBitCnt : unsigned( 3 downto  0 );
signal packetValid : std_logic;

-- TX signals.
signal txComb : std_logic_vector( 7 downto 0 );
signal txBuff : std_logic_vector( 7 downto 0 );
signal startTX : std_logic;
signal txActive : std_logic;
signal txPacketDone : std_logic;
signal txOut : std_logic;
signal txBitTimeCnt : unsigned( bits4us - 1 downto 0 );
signal txBitCnt : unsigned( 3 downto 0 );
signal lastPacketComb : std_logic;
signal lastPacketSeq : std_logic;
signal bitSendDone : std_logic;

-- FSM
type FSM_States is ( idle, waitTX, setButResp1, setButResp2, setButResp3,
                     setButResp4, setStatResp1, setStatResp2,setStatResp3,
                     waitAfterRX, getAddr1, getAddr2, writeMem, setCRCResp, 
                     readMem );
signal curState : FSM_States;
signal nextState : FSM_States;
signal stateAfterWait : FSM_States;
signal stateAfterWaitSeq : FSM_States;

signal waitCnt : unsigned( bits5us - 1 downto 0 );
signal waitCntRst : std_logic;
signal waitCntComp : unsigned( waitCnt'length - 1 downto 0 );
signal waitCntReached : std_logic;

-- Signals for the memory controller.
signal memAddr : std_logic_vector( 14 downto 0 );
signal memWriteEn : std_logic;
signal memDataIn : std_logic_vector( 7 downto 0 );
signal memDataOut : std_logic_vector( 7 downto 0 );
signal memDataInValid : std_logic;
signal memDataOutValid : std_logic;
signal memBusy : std_logic;
signal memSPIDataTX : std_logic_vector( 7 downto 0 );
signal memSPIDataTXValid : std_logic;
signal memSPIDataRX : std_logic_vector( 7 downto 0 );
signal memSPIDataRXValid : std_logic;
signal memSPIintSS : std_logic;

-- A flag signaling if a new data was read from FRAM.
signal lastMEMData : std_logic_vector( 7 downto 0 );
signal newDataRead : std_logic;
signal newDataReadAck : std_logic;


begin

   -- For now, only no pak status.
  --lastStatus <= noPakStatus;

  --Capture input signals.
  inSample:process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      serSyn <= '1';
      serSynDel <= '1';
      lastPacketSeq <= '0';
      lastMEMData <= ( others => '0' );
    elsif rising_edge( clk ) then
      serSyn <= serialDatIn;
      serSynDel <= serSyn;
      lastPacketSeq <= lastPacketComb;
       
       if ( memDataOutValid = '1' ) then
         lastMEMData <= memDataOut;
       end if;
    end if;
  end process;
   
  -- Instantiate the FRAM memory controller.
  framController : entity work.FRAM_SPI_controller( Behavioral )
    generic map(
      clkfreq => clkfreq
    )
    
    port map(
      dataIn => memDataIn,
      writeEn => memWriteEn,
      addr => memAddr,
      validIn => memDataInValid,
      dataOut => memDataOut,
      validOut => memDataOutValid,
      busy => memBusy,
      spiDataIn => memSPIDataRX,
      spiDataInValid => memSPIDataRXValid,
      spiDataOut => memSPIDataTX,
      spiDataOutValid => memSPIDataTXValid,
      spiSS => memSPIintSS,
      clk => clk,
      rst => rst
    );
    
  -- The FRAM address.
  memAddr <= lastAddr( 9 downto 0 ) & std_logic_vector( packetCntDel( 4 downto 0 ) );
   
  -- Process for setting the new data flag.
  newDataFlagProc:process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      newDataRead <= '0';
    elsif rising_edge( clk ) then
      if ( memDataOutValid = '1' ) then
        newDataRead <= '1';
        elsif ( newDataReadAck = '1' ) then
          newDataRead <= '0';
      end if;
    end if;
  end process;

  -- The new data is acknowledged when we move from readMem to the 
  -- TX state.
  newDataReadAck <= '1' when ( curState = readMem and nextState = waitTX ) else
                   '0';
   
  -- The FRAM IN data.
  memDataIn <= n64RXPacket;

  -- And the SPI controller for the FRAM.
  framSPI : entity work.spimaster( Behavioral )
    generic map(
       clkfreq => clkfreq, --in kHz
       spiclkfreq => to_unsigned( 2000, 17 ), --in kHz
       packetsize => to_unsigned( 8, 4 ), -- in bit
       ss_default => '1'
    )
    
    port map(
       clk => clk,
       rst => rst,
       ss_in => memSPIintSS,
       din_valid => memSPIDataTXValid,
       din => memSPIDataTX,
       miso => memMISO,
       dout => memSPIDataRX,
       dvalid => memSPIDataRXValid,
       mosi => memMOSI,
       ss_out => memSS,
       sclk => memSCLK      
    );
   
  
  --Processes to sample the N64 serial to single bits.
  bitTimeCntRst <= rst or serFall;
  -- Do not capture our own TX signals.
  serFall <= ( not SerSyn and SerSynDel ) and not txActive;
  bitTimeCntZero <= '1' when bitTimeCnt = 0 else
                    '0';
  serValueValid <= bitTimeCntZero and not bitTimeCntZeroDel;
  serValue <= serSyn;
  process( clk, bitTimeCntRst ) is
  begin
    if ( bitTimeCntRst = '1' ) then
      bitTimeCnt <= to_unsigned( cycles2us, bitTimeCnt'length );
      bitTimeCntZeroDel <= '0';
    elsif rising_edge( clk ) then
      bitTImeCntZeroDel <= bitTimeCntZero;
      if ( bitTimeCnt = 0 ) then
        bitTimeCnt <= ( others => '0' );
      else
        bitTimeCnt <= bitTimeCnt - 1;
      end if;
    end if;
  end process;
  
  --Update the buttons or status memory memory when a new one was received.
  process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      lastButtons <= ( others => '0' );
      lastButtonsRXBuffer <= ( others => '0' );
      lastStatusRXBuffer <= noPakStatus;
      lastStatus <= noPakStatus;
      curPakStatus <= '0';
    elsif rising_edge( clk ) then
      -- Check RX is valid and has the right length (4 byte + 1 status byte from the receiver).
      if ( rxDataValid = '1' and rxLen = 5 ) then
        -- We only want the upper 4 bytes.
        lastButtonsRXBuffer <= rxData( rxData'length - 1 downto rxData'length - 32 );
        -- Received new status.
      elsif ( rxDataValid = '1' and rxLen = 2 ) then
        -- Check if rumble mode and plugged in.
        if ( rxData( rxData'length - 4 ) = '1' and rxData( rxData'length - 8 ) = '1' ) then
          lastStatusRXBuffer <= pakStatus;
            curPakStatus <= '0';
          -- Memory pack?
        elsif ( rxData( rxData'length - 4 ) = '0' ) then
          lastStatusRXBuffer <= pakStatus;
          curPakStatus <= '1';
        else
          lastStatusRXBuffer <= noPakStatus;
          curPakStatus <= '0';
        end if;
      end if;
     
      -- If we are not currently sending something to the N64,
      -- update the actual memory.
      if ( curState /= waitTX ) then
        lastButtons <= lastButtonsRXBuffer;
        lastStatus <= lastStatusRXBuffer;
      end if;
     
    end if;
  end process;
  
  -- Packetize the single bits.
  process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      packetBitCnt <=  ( others => '0' );
      packetTimeCnt <= to_unsigned( cycles5us, packetTimeCnt'length );
      n64RXPacket <= ( others => '0' );
    elsif rising_edge( clk ) then
      if ( serValueValid = '1' ) then
        -- Shift in the serial.
        n64RXPacket( 0 ) <= serValue;
       n64RXPacket( 7 downto 1 ) <= n64RXPacket( 6 downto 0 );
       -- Reset the timer.
       packetTimeCnt <= to_unsigned( cycles5us, packetTimeCnt'length );
       -- Increase the timer.
       packetBitCnt <= packetBitCnt + 1;
      else
        -- Decrease timer.
        if ( packetTimeCnt = 0 ) then
          packetTimeCnt <= ( others => '0' );
          packetBitCnt <= ( others => '0' );
       else
          packetTimeCnt <= packetTimeCnt - 1;
          -- If we reached a full packet, reset the bit counter.
          if ( packetBitCnt = 8 ) then
            packetBitCnt <= ( others => '0' );
          end if;
        end if;
      end if;
    end if;
  end process;

  packetValid <= '1' when packetBitCnt = 8 else
                 '0';
            
  -- TX stuff.
  bitSendDone <= '1' when txBitTimeCnt = cycles4us else
                 '0';
            
  txPacketDone <= '1' when ( txBitCnt = 7 and lastPacketSeq = '0' and bitSendDone = '1' ) else
                  '1' when ( txBitCnt = 8 and lastPacketSeq = '1' and bitSendDone = '1' ) else
                  '0';
  
  process( clk, rst ) is 
  begin
    if ( rst = '1' ) then
      txBuff <= ( others => '0' );
      txBitTimeCnt <= ( others => '0' );
      txBitCnt <= ( others => '0' );
      txActive <= '0';
    
    elsif rising_edge( clk ) then
      if ( startTX = '1' ) then
        txBuff <= txComb;
        txBitTimeCnt <= ( others => '0' );
        txBitCnt <= ( others => '0' );
        txActive <= '1';
     
      elsif ( bitSendDone = '1' ) then
        -- Shift by one (to the left). We fill it up with 
        -- '1's for the last packet to write a stop bit.
        txBuff( 0 ) <= '1';
        txBuff( 7 downto 1 ) <= txBuff( 6 downto 0 );
        -- And reset the time counter.
        txBitTimeCnt <= ( others => '0' );
      
        -- Increase the bit counter.
        txBitCnt <= txBitCnt + 1;
      
      elsif ( bitSendDone = '0' and txActive = '1' ) then
        -- Increase the timer.
        txBitTimeCnt <= txBitTimeCnt + 1;
     
      end if;
     
      -- Bit counter.
      -- Check if we are this is the last packet. If yes,
      -- write a stop bit (simply one more round).
      if ( bitSendDone = '1' and txBitCnt = 8 and lastPacketSeq = '1' ) then
        txActive <= '0'; 
      end if;
    end if;
  end process;
  
  -- TX output.
  process( txBuff( 7 ), txBitTimeCnt ) is
  begin
    -- Send a 0.
    if ( txBuff( 7 ) = '0' ) then
      if ( txBitTimeCnt >= cycles3us ) then
        txOut <= '1';
      else
        txOut <= '0';
      end if;
    else 
      -- Send a 1.
      if ( txBitTimeCnt >= cycles1us ) then
        txOut <= '1';
      else
        txOut <= '0';
      end if;
    end if;
  end process;
   
   
  -- Memory / Rumble pack writing processes.
  -- A timeout counter.
  writeBytesTimeout:process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      timeoutCnt <= ( others => '0' );
    elsif ( rising_edge( clk ) ) then
      if ( timeoutCntRst = '1' ) then
        timeoutCnt <= ( others => '0' );
       
      else
        if ( timeoutCnt /= cycles1050us ) then
          timeoutCnt <= timeoutCnt + 1;
        end if;
      end if;
    end if;
  end process;

  timeoutCntReached <= '1' when ( timeoutCnt = cycles1050us ) else '0';
   
  -- We reset the timeoutCnt when we enter the writeMode state.
  timeoutCntRst <= '1' when ( nextState = writeMem and curState /= writeMem ) else '0';

  -- Set the writeMode according to the last address.
  writeMode <= '1' when ( unsigned( lastAddr ) < unsigned( rumbleAddrStart ) ) else '0';

  -- Rumble mode.
  rumbleMode <= rumbleOn;

  -- Check if a new byte has to be written or read.
  writeReadByte:process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      packetCnt <= ( others => '0' );
      packetCntDel <= ( others => '0' );
      rumbleLatch <= ( others => '0' );
      rumbleOn <= '0';
      memWriteEn <= '0';
      memDataInValid <= '0';
    elsif ( rising_edge( clk ) ) then
      packetCntDel <= packetCnt;
     
      if ( packetCntRst = '1' ) then
        packetCnt <= ( others => '0' );
        memWriteEn <= '0';
        memDataInValid <= '0';
         
      else        
       
        if ( curState = writeMem and packetValid = '1' ) then
          -- Increase the counter.
          packetCnt <= packetCnt + 1;
           
          -- We have to do a write.
          if ( writeMode = '0' ) then
            -- Off area write.
            if ( curPakStatus = '0' ) then
              -- Rumble pack. Save to latch.
              rumbleLatch <= n64RXPacket;
              memWriteEn <= '0';
              memDataInValid <= '0';
              -- The rumble mode is modified when adress 0xC000 is accessed.
              if ( lastAddr = "11000000000" ) then
                if ( n64RXPacket = "00000000" ) then
                  rumbleOn <= '0';
                 else
                   rumbleOn <= '1';
                 end if;
               end if;
             else
               -- Memory pack write to latch. Do not do anything.
               memWriteEn <= '0';
               memDataInValid <= '0';
             end if;
           else
             -- Mem area write.
             if ( curPakStatus = '0' ) then
               -- Rumble pack. Do nothing.
               memWriteEn <= '0';
               memDataInValid <= '0';
             else
               -- Memory pack write.
               memWriteEn <= '1';
               memDataInValid <= '1';
             end if;
           end if;
           
         elsif ( curState = getAddr2 and nextState = waitAfterRX and stateAfterWaitSeq = readMem ) or
               ( curState = readMem and nextState = waitTX and packetCnt /= 32 ) then
           -- We have to perform a read.
           
           -- Increase the counter.
           packetCnt <= packetCnt + 1;
           
           if ( writeMode = '0' ) then
             -- Off area read.
             if ( curPakStatus = '0' ) then
               -- Rumble pack. Don't do anything.
               memWriteEn <= '0';
               memDataInValid <= '0';
             else
               -- Memory pack read, off area. do not to anything.
               memWriteEn <= '0';
               memDataInValid <= '0';
             end if;
           else
             -- Mem area read.
             if ( curPakStatus = '0' ) then
               -- Rumble pack. Do nothing.
               memWriteEn <= '0';
               memDataInValid <= '0';
             else
               -- Memory pack read.
               memWriteEn <= '0';
               memDataInValid <= '1';
             end if;
           end if;
         
         elsif ( curState = readMem and nextState = waitTX ) then
           -- Just increase.
           packetCnt <= packetCnt + 1;
           
         else
           -- Do not do anything.
           memWriteEn <= '0';
           memDataInValid <= '0';
         end if;
       end if;
     end if;
   end process;
   
  -- We reset the packetCnt the time during idle.
  packetCntRst <= '1' when ( curState = idle ) else
                   '0';
   
  -- Process to update the last address.
  lastAddrUpdate : process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      lastAddr <= ( others => '0' );
     
    elsif ( rising_edge( clk ) ) then
      if ( packetValid = '1' ) then
        if ( curState = getAddr1 ) then
          lastAddr( 10 downto 3 ) <= n64RXPacket;
        elsif ( curState = getAddr2 ) then
          lastAddr( 2 downto 0 ) <= n64RXPacket( 7 downto 5 );
        end if;
      end if;     
    end if;
  end process;
   
  -- The CRC8 module.
  crc8unit : entity work.crc8( Behavioral )
    generic map(
      postXOR => '0',
      initValue => ( others => '0' ),
      poly => x"85" )
     
    port map(
      dataIn => crcDatIn,
      enable => crc8En,
      clk => clk,
      rst => crc8Rst,
      crcOut => crc8Result );
     
  -- If we are in writeMem, the input is the n64RXPacket.
  -- ...if readMem, then use the data to be sent out.
  crcDatIn <= n64RXPacket when ( curState = writeMem ) else
              txComb when ( curState = readMem ) else
              ( others => '0' );
     
  -- The CRC8 module is resetted either by the global reset or when
  -- the packetCnt is rst.
  crc8Rst <= rst or packetCntRst;

  -- A new byte is fed into the CRC8 module whenever a new packet
  -- arrived and we are in writeMem state.
  -- If we are in the readMem state, use the txStart Signal.
  crc8En <= '1' when ( curState = writeMem and packetValid = '1' ) else 
            '1' when ( curState = readMem and startTX = '1' ) else 
            '0';
   
  
  -- FSM processes.
  
  stateUpd:process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      curState <= idle;
      stateAfterWaitSeq <= idle;
    elsif rising_edge( clk ) then
      curState <= nextState;
      stateAfterWaitSeq <= stateAfterWait;
    end if;
  end process;
  
  nextStateLogic:process( packetValid, n64RXPacket, stateAfterWaitSeq, curState,
                          txPacketDone, waitCntReached, packetCnt, timeoutCntReached,
                          curPakStatus, newDataRead, writeMode ) is
  begin
    case curState is
      when idle =>
        -- Check if there is a new packet.
        if ( packetValid = '1' ) then
          -- Check what packet.
          if ( n64RXPacket = cmdReadButtons ) then
            stateAfterWait <= setButResp1;
            nextState <= waitAfterRX;
             
          elsif ( n64RXPacket = cmdReadStatus ) then
            stateAfterWait <= setStatResp1;
            nextState <= waitAfterRX;
             
          elsif ( n64RXPacket = cmdWriteMem ) then
            nextState <= getAddr1;
            stateAfterWait <= writeMem;
             
          elsif ( n64RXPacket = cmdReadMem ) then
            nextState <= getAddr1;
            stateAfterWait <= readMem;
             
          else
            nextState <= idle;
            stateAfterWait <= idle;
          end if;
        else
          nextState <= idle;
          stateAfterWait <= idle;
        end if;
      
        when setButResp1 => 
          stateAfterWait <= setButResp2;
          nextState <= waitTX;
     
        when setButResp2 => 
          stateAfterWait <= setButResp3;
          nextState <= waitTX;
      
        when setButResp3 => 
          stateAfterWait <= setButResp4;
          nextState <= waitTX;
      
        when setButResp4 => 
          stateAfterWait <= idle;
          nextState <= waitTX;
      
        when setStatResp1 =>
          stateAfterWait <= setStatResp2;
          nextState <= waitTX;
      
        when setStatResp2 =>
          stateAfterWait <= setStatResp3;
          nextState <= waitTX;
      
        when setStatResp3 =>
          stateAfterWait <= idle;
          nextState <= waitTX;
      
        when waitTX =>
          stateAfterWait <= stateAfterWaitSeq;
          if ( txPacketDone = '1' ) then
            nextState <= stateAfterWaitSeq;
          else
            nextState <= waitTX;
          end if;
      
        when waitAfterRX =>
          if ( waitCntReached = '1' ) then
            nextState <= stateAfterWaitSeq;
            stateAfterWait <= waitAfterRX;
          else
            nextState <= waitAfterRX;
            stateAfterWait <= stateAfterWaitSeq;
          end if;
         
        when getAddr1 =>
          if ( packetValid = '1' ) then
            nextState <= getAddr2;
            stateAfterWait <= stateAfterWaitSeq;
           
          else
            nextState <= getAddr1;
            stateAfterWait <= stateAfterWaitSeq;
          end if;
         
        when getAddr2 =>
          if ( packetValid = '1' ) then
            -- In case of reading, we want to first delay a bit.
            if ( stateAfterWaitSeq = readMem ) then
              nextState <= waitAfterRX;
              stateAfterWait <= readMem;
            else
              nextState <= stateAfterWaitSeq;
              stateAfterWait <= getAddr2;
            end if;

          else
            nextState <= getAddr2;
            stateAfterWait <= stateAfterWaitSeq;
          end if;
         
        when writeMem =>
          if ( packetCnt = 32 ) then
            nextState <= waitAfterRX;
            stateAfterWait <= setCRCResp;
           
          elsif ( timeoutCntReached = '1' ) then
            nextState <= idle;
            stateAfterWait <= writeMem;
           
          else
            nextState <= writeMem;
            stateAfterWait <= writeMem;
          end if;
         
        when readMem =>
          -- When packetCnt = 32, we still need to send the 31th byte.
          if ( packetCnt = 33 ) then
            nextState <= setCRCResp;
            stateAfterWait <= readMem;
           
          else
            -- Check if we actually have to read from
            -- memory or only put out the latch value.
            if ( curPakStatus = '1' and writeMode = '1' ) then
              -- Memory pack.
              if ( newDataRead = '1' ) then
                nextState <= waitTX;
                stateAfterWait <= readMem;
              else
                nextState <= readMem;
                stateAfterWait <= readMem;
              end if;
            else 
              -- Rumble latch or off area read..
              nextState <= waitTX;
              stateAfterWait <= readMem;
            end if;
          end if;
         
        when setCRCResp =>
          nextState <= waitTX;
          stateAfterWait <= idle;
     
        when others =>
          -- Dummy.
          nextState <= idle;
     
    end case;
  end process;
  
  outputLogic:process( curState, lastButtons, lastStatus, 
    lastPacketSeq, crc8Result, writeMode, rumbleLatch, curPakStatus,
    newDataRead, lastMemData, packetCnt ) is
  begin
  
    case curState is
      when idle =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= ( others => '0' );
        startTX <= '0';
        lastPacketComb <= '0';
      
      when setButResp1 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= lastButtons( 31 downto 24 );
        startTX <= '1';
        lastPacketComb <= '0';
      
      when setButResp2 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= lastButtons( 23 downto 16 );
        startTX <= '1';
        lastPacketComb <= '0';
      
      when setButResp3 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= lastButtons( 15 downto 8 );
        startTX <= '1';
        lastPacketComb <= '0';
      
      when setButResp4 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= lastButtons( 7 downto 0 );
        startTX <= '1';
        lastPacketComb <= '1';
      
      when waitTX =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= ( others => '0' );
        startTX <= '0';
        lastPacketComb <= lastPacketSeq;
      
      when waitAfterRX =>
        waitCntRst <= '0';
        waitCntComp <= to_unsigned( cycles5us, waitCntComp'length );
        txComb <= ( others => '0' );
        startTX <= '0';
        lastPacketComb <= '0';
      
      when setStatResp1 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= lastStatus( 23 downto 16 );
        startTX <= '1';
        lastPacketComb <= '0';
      
      when setStatResp2 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= lastStatus( 15 downto 8 );
        startTX <= '1';
        lastPacketComb <= '0';
      
      when setStatResp3 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= lastStatus( 7 downto 0 );
        startTX <= '1';
        lastPacketComb <= '1';
         
      when getAddr1 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= ( others => '0' );
        startTX <= '0';
        lastPacketComb <= '0';
         
      when getAddr2 =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= ( others => '0' );
        startTX <= '0';
        lastPacketComb <= '0';
         
      when writeMem =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= ( others => '0' );
        startTX <= '0';
        lastPacketComb <= '0';
         
      when setCRCResp =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= crc8Result;
        startTX <= '1';
        lastPacketComb <= '1';
         
      when readMem =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        if ( curPakStatus = '0' ) then
          if ( writeMode = '0' ) then
            txComb <= rumbleLatch;
          else
            txComb <= ( others => '0' );
          end if;
        elsif ( writeMode = '0' ) then
          txComb <= ( others => '0' );
        elsif ( newDataRead = '1' ) then
          txComb <= lastMEMData;
        else
          txComb <= ( others => '0' );
        end if;
      
        -- Only send if we stil need.
        if ( packetCnt /= 33 ) then
          if ( writeMode = '0' or curPakStatus = '0' ) then
            startTX <= '1';
          elsif ( newDataRead = '1' ) then
            startTX <= '1';
          else
            startTX <= '0';
          end if;
        else
          startTX <= '0';
        end if;
      lastPacketComb <= '0';
      
      when others =>
        waitCntRst <= '1';
        waitCntComp <= ( others => '0' );
        txComb <= ( others => '0' );
        startTX <= '0';
        lastPacketComb <= '0';

    end case;
  
  end process;
  
  -- Wait counter.
  process( clk, rst ) is
  begin
    if ( rst = '1' ) then
      waitCnt <= ( others => '0' );
    elsif rising_edge( clk ) then
      if ( waitCntRst = '1' ) then
        waitCnt <= ( others => '0' );
      else
        waitCnt <= waitCnt + 1;
      end if;
    end if;
  end process;
  
  waitCntReached <= '1' when waitCnt >= waitCntComp else
                    '0';
              
  -- TriState Enable.
  process( txActive, txOut ) is
  begin
    if ( txActive = '1' ) then
      if ( txOut = '1' ) then
        serialDatOut <= 'Z';
      else
        serialDatOut <= '0';
      end if;
    else
      serialDatOut <= 'Z';
    end if;
  end process;
end Behavioral;

