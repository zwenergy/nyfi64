library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.numeric_std.all;

entity crc8 is
  generic( postXOR : std_logic := '0';
           initValue : std_logic_vector( 7 downto 0 ) := ( others => '0' );
           poly : std_logic_vector( 7 downto 0 ) := x"85" -- Implicit 9th 1 bit.
        );
        
  port( dataIn : in std_logic_vector( 7 downto 0 );
        enable : in std_logic;
        clk : in std_logic;
        rst : in std_logic;
        crcOut : out std_logic_vector( 7 downto 0 ) );
        
end crc8;


architecture Behavioral of crc8 is
signal crcOut_int : std_logic_vector( crcOut'length - 1 downto 0 );
begin

  -- Capture input and calculate CRC.
  process( clk, rst ) is 
  variable tmpCRC : std_logic_vector( 7 downto 0 );
  variable varCRC : std_logic_vector( 7 downto 0 );
  begin
    if ( rst = '1' ) then
      varCRC := initValue;
    elsif ( rising_edge( clk ) ) then
      if ( enable = '1' ) then
        for i in 7 downto 0 loop
          tmpCRC( 0 ) := ( dataIn( i ) xor varCRC( 7 ) ) and poly( 0 );
          for j in 1 to 7 loop
            tmpCRC( j ) := ( ( dataIn( i ) xor varCRC( 7 ) ) and poly( j ) ) xor varCRC( j - 1 );
          end loop;
          varCRC := tmpCRC;
        end loop;
      
        -- And the XOR.
        for i in 0 to 7 loop
          varCRC( i ) := varCRC( i ) xor postXOR;
        end loop;
      end if;
    end if;
  crcOut_int <= varCRC;
  end process;

  crcOut <= crcOut_int;
end Behavioral;

