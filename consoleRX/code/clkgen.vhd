library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.math_real.ALL;
use IEEE.NUMERIC_STD.ALL;


entity clkgen is
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
    
end clkgen;

architecture Behavioral of clkgen is
constant cntMax : unsigned( 16 downto 0 ) := ( clkfreqin / clkfreqout ) - 1;
constant cntMaxHalf : unsigned( 16 downto 0 ) := ( '0' & cntMax( 16 downto 1 ) );
signal cnt : unsigned( cntMax'length - 1 downto 0 );
signal clk_out_int : std_logic;
begin

  divProc : process( clk_in, rst )
  begin
    if ( rst = '1' ) then
      clk_out_int <= rst_val;
      cnt <= ( others => '0' );
    elsif ( rising_edge( clk_in ) ) then
      if ( cnt < cntMaxHalf ) then
        cnt <= cnt + 1;
        clk_out_int <= rst_val;
      elsif ( cnt = cntMax ) then
        cnt <= ( others => '0' );
        clk_out_int <= not rst_val;
      else
        cnt <= cnt + 1;
        clk_out_int <= not rst_val;
      end if;
        
    end if;
  end process;
  
  clk_out <= clk_out_int;

end Behavioral;

