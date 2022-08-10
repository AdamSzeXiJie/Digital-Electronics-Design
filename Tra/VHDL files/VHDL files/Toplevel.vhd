--===================================================================
--  Example VHDL for HET202 - Electronic Die
--
--  Revision History
--===================================================================
--  6/ 5/06 - Created - pgo
--===================================================================
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all; -- Need arithmetic

entity Toplevel is
    Port ( Clock    : in  std_logic; -- system clock
           Reset    : in  std_logic; -- reset button on FPGA board
           debugLed : out std_logic; -- debug LED on FPGA board
           leds     : out std_logic_vector(2 downto 0); -- counter outputs
           button   : in  std_logic; -- run button
           dieLeds  : out std_logic_vector(3 downto 0)  -- die pattern LEDs
           );
end Toplevel;

architecture behaviour of Toplevel is 

-- Synchronized button inputs
signal rollButton : std_logic;

-- Die roll
signal dieCount   : std_logic_vector(2 downto 0);

begin

   -- For debug - show reset on debug LED
   debugLed <= reset;

   -- For debug - show dieCount on LEDs
   leds(2 downto 0) <= dieCount;

   --=====================================
   -- Input button synchronization
   -- Required to prevent metastability, and input uncertainty due to skew
   buttonSync:
   process( Reset, Clock, button )
   begin

      if (reset = '1') then
         rollButton  <= '0';
      elsif (clock'event and (clock = '1')) then
         rollButton  <= button;
      end if;
   end process buttonSync;

   --=====================================
   -- Instantiate decoder
   decoder:
   entity work.DieDecoder 
       Port Map ( count => dieCount,
                  LEDs  => dieLeds );

   --=====================================
   -- Instantiate counter
   counter:
   entity work.dieCounter
       Port Map ( reset => Reset,
                  clock => Clock,
                  roll  => rollButton,
                  count => dieCount );

end architecture behaviour;
