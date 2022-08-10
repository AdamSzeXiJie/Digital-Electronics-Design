--================================================================================
-- TrafficTopLevel.vhd
--
-- Traffic light system to control an intersection
--
--
--================================================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity Traffic is
	Port ( 	Reset : in STD_LOGIC;
				Clock : in STD_LOGIC;

				-- Car and pedestrian buttons
				Train : in STD_LOGIC; -- Train on EW road
				CarNS : in STD_LOGIC; -- Car on NS road
				PedNS : in STD_LOGIC; -- Pedestrian moving NS (crossing EW road)

				-- Light control
				HLights : out STD_LOGIC_VECTOR (1 downto 0); -- controls EW lights
				VLights : out STD_LOGIC_VECTOR (1 downto 0); -- controls NS lights
				Motor    : out STD_LOGIC_VECTOR (3 downto 0)   -- controls Motor
			);
end entity Traffic;

architecture Behavioral of Traffic is
	COMPONENT SyncButton
		PORT ( reset : in  STD_LOGIC;
           clock : in  STD_LOGIC;
           TrainButton : in  STD_LOGIC; -- Train Button Input --
           CarButton : in  STD_LOGIC; -- Car Button Input --
           PedButton : in  STD_LOGIC; -- Pedestrain Button Input --
           Counter : in  STD_LOGIC_VECTOR (4 downto 0); -- Timer --
           TrainAction : out  STD_LOGIC; -- Tran Button Output --
           CarWaitOutput : out  STD_LOGIC; -- Car Output When Train Is And Is Not Passing Through --
           PedWaitOutput : out  STD_LOGIC -- Pedestrain Output When Train Is And Is Not Passing Through --
			  );
		END COMPONENT;
		
	COMPONENT TimerMealyMachine
		PORT ( reset : in  STD_LOGIC;
           clock : in  STD_LOGIC;
			  TrainButton : in  STD_LOGIC; -- Train Button Input --
			  CarButton : in  STD_LOGIC; -- Car Button Input --
			  PedButton : in  STD_LOGIC; -- Pedestrain Button Input --
			  Found : out  STD_LOGIC -- Output To Reset Timer --
			  );
		END COMPONENT;
		
	COMPONENT Timer
		PORT ( reset : in  STD_LOGIC;
           clock : in  STD_LOGIC;
           timer_reset : in  STD_LOGIC; -- To Reset Timer --
			  FourHzPulse : out  STD_LOGIC; -- Four Hz Pulse --
           MotorPulse : out STD_LOGIC; -- Pulse For Stepper Motor --
			  CounterOutput : out  STD_LOGIC_VECTOR (4 downto 0); -- Timer Output --
			  PedTimer : out STD_LOGIC -- Timer For Pedestrian To Cross The Road --
			  );
		END COMPONENT;
		
	COMPONENT MotorController
		PORT ( reset : in  STD_LOGIC;
           MotorPulse : in  STD_LOGIC; -- Pulse For Stepper Motor --
           Enable : in  STD_LOGIC; -- To Enable The Motor To Turn --
           Clockwise : in  STD_LOGIC; -- To Choose Direction To Turn --
           MotorOutput : out  STD_LOGIC_VECTOR (3 downto 0) -- Output for Motor --
			  );
		END COMPONENT;
		
	COMPONENT TrafficLightsMooreMachine
		PORT ( reset : in  STD_LOGIC;
           clock : in  STD_LOGIC;
			  FourHzPulse : in  STD_LOGIC; -- 4Hz Pulse --
			  Flash : in  STD_LOGIC; -- Train Button is Pressed --
			  TrafficGreen : in  STD_LOGIC; -- Car Button is Pressed --
			  PedGreen : in  STD_LOGIC; -- Pedestrian Button is Pressed --
			  MotorEnable : out  STD_LOGIC; -- Enable The Stepper Motor To Rotate --
			  MotorClockwise : out  STD_LOGIC; -- Direction Of Rotation For Stepper Motor--
			  PedHold  : in  STD_LOGIC; -- Hold Pedestrian Input --
           HTrafficLightOutput : out  STD_LOGIC_VECTOR (1 downto 0); -- Output for Horizontal Traffic Light --
           VTrafficLightOutput : out  STD_LOGIC_VECTOR (1 downto 0) -- Output for Vertical Traffic Light --
			  );
		END COMPONENT;
		
	signal TrainInputButton : std_logic;
	signal CarInputButton : std_logic;
	signal PedInputButton : std_logic;
	signal ResetTimer : std_logic;
	signal MotorEnableOutput : std_logic;
	signal MotorClockwiseOutput : std_logic;
	signal MotorPulseOutput : std_logic;
	signal PedestrianTimer : std_logic;
	signal FourHzOutput : std_logic;
	signal TimerOutput : std_logic_vector (4 downto 0);
		
	begin
	
	ButtonInput : SyncButton PORT MAP (
		reset => Reset,
      clock => Clock,
      TrainButton => Train,
      CarButton => CarNS,
      PedButton => PedNS,
      Counter => TimerOutput,
      TrainAction => TrainInputButton,
      CarWaitOutput => CarInputButton,
      PedWaitOutput => PedInputButton
	);
	
	MealyMachineTimer : TimerMealyMachine PORT MAP (
		reset => Reset,
      clock => Clock,
		TrainButton => TrainInputButton,
		CarButton => CarInputButton,
		PedButton => PedInputButton,
		Found => ResetTimer
	);
	
	SystemTimer : Timer PORT MAP (
		reset => Reset,
      clock => Clock,
      timer_reset => ResetTimer,
		FourHzPulse => FourHzOutput,
      MotorPulse => MotorPulseOutput,
		CounterOutput => TimerOutput,
		PedTimer => PedestrianTimer
	);
	
	StepperModule : MotorController PORT MAP (
		reset => Reset,
      MotorPulse => MotorPulseOutput,
      Enable => MotorEnableOutput,
      Clockwise => MotorClockwiseOutput,
      MotorOutput => Motor
	);
	
	TrafficLight : TrafficLightsMooreMachine PORT MAP (
		reset => Reset,
      clock => Clock,
		FourHzPulse => FourHzOutput,
		Flash => TrainInputButton,
		TrafficGreen => CarInputButton,
	   PedGreen => PedInputButton,
		MotorEnable => MotorEnableOutput,
		MotorClockwise => MotorClockwiseOutput,
		PedHold => PedestrianTimer,
      HTrafficLightOutput => HLights,
      VTrafficLightOutput => VLights
	);
	--Insert your code here

end architecture Behavioral;