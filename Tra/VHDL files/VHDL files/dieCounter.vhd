--===================================================================
-- DieCounter.vhdl
--
-- A counter to produce a roll in the range 1 .. 6 (like a die)
--===================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity dieCounter is
    Port ( reset : in  std_logic;
           clock : in  std_logic;
           roll  : in  std_logic;
           count : out std_logic_vector(2 downto 0));
end entity dieCounter;

architecture Behavioural of dieCounter is

signal dieCount : std_logic_vector(2 downto 0);

begin
   count <= dieCount;

   -- Produce dieCount sequence from 1 ... 6
   counter:
   process( reset, clock )
   begin
      if (reset = '1') then
         dieCount <= "001";        -- start count from 1
      elsif (clock'event and (clock = '1')) then
         if (roll = '1') then      -- rolling
            if (dieCount >= "110") then
               dieCount <= "001";  -- wrap count
            else
               dieCount <= dieCount + "001";
            end if;
         end if;
      end if;
   end process;

end architecture Behavioural;
