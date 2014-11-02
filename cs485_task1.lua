--[[
	CS 485 Project 3 Part 1
	Authors: Chris Beckley and Laura Hovatter 
	Program: Move to the given (x_move,y_move) position and rotate
			 to face the given (x_rotate,y_rotate) position.
--]]

--###################################################################################################--
--#################################### Global Variable Setup ########################################--
--###################################################################################################--

-- darwin = our robotic friend
-- (x_move,y_move) = the point to move to
-- (x_rotate,y_rotate) = the point to rotate to face towards
darwin = require('hfaDriver')
require "os"	-- Used for working with the clock and time functions
require "math"  -- Used for trigonometric functions
x_move = 0.0
y_move = 0.0
x_rotate = 1.0
y_rotate = -1.0
time_start = 0
desired_angle = 0.0
current_angle = 0.0
print_count = 0

--###################################################################################################--
--#################################### Behavior Definitions #########################################--
--###################################################################################################--



-- behaviour to find the goal/find our position?
findPositionStart = function(hfa)
print("finding the goal")
-- looks around for the goal, may or may not see it so we may need to move arbitrarily until we find one of them...
darwin.stop();
time_start = os.clock();
print("start time: " .. time_start);
darwin.lookGoal(); 
--darwin.setVelocity(0, 0,1); -- rad/sec for angular velocity
end
findPositionGo = function(hfa)
end
findPositionStop = function(hfa)
--Found the goal, stop so we can calculate our position
end


-- face a certain position
faceDirectionStart = function(hfa)
print("starting face direction")
--Stop so we can calculate our position
darwin.stop();
-- Grab position to go to in body frame
local toGoPosX = x_move - wcm.get_pose().x;
local toGoPosY = y_move - wcm.get_pose().y;
current_angle = wcm.get_pose().a;

-- Calculate the angle to position
desired_angle = math.atan(toGoPosY / toGoPosX);
local delta_angle = desired_angle - current_angle;
print("desired angle: " .. desired_angle);
print("current angle: " .. current_angle);
print("delta angle: " .. delta_angle);
-- Set velocity based on angle to turn
if (delta_angle < 0) then
	darwin.setVelocity(0,0,-10);
	print("setting omega to -10");
else
	darwin.setVelocity(0,0,10);
	print("setting omega to 10");
end
end

faceDirectionGo = function(hfa)
current_angle = wcm.get_pose().a;
end

faceDirectionStop = function(hfa)
darwin.setVelocity(0,0,0);
darwin.stop();
end



--walk to a certain position, given we know our position on the field already (we found a goal). 
walkToPosStart = function(hfa)
print("starting to walk to (" .. x_move .. "," .. y_move .. ")");
darwin.setVelocity(0.05,0,0);
end

walkToPosGo = function(hfa)
local toGoPosX = x_move - wcm.get_pose().x;
local toGoPosY = y_move - wcm.get_pose().y;
current_angle = wcm.get_pose().a;

-- Calculate the angle to position
desired_angle = math.atan(toGoPosY / toGoPosX);
end

walkToPosStop = function(hfa)
darwin.stop();
print("Arrived at destination!")
print("My position is (" .. wcm.get_pose().x .. "," .. wcm.get_pose().y .. ")");
end

-- Face the goal X,Y position
faceGoalStart = function(hfa)
print("starting to face towards goal")
--Stop so we can calculate our position
darwin.stop();
-- Grab position to go to in body frame
local toGoPosX = x_rotate - wcm.get_pose().x;
local toGoPosY = y_rotate - wcm.get_pose().y;
current_angle = wcm.get_pose().a;

-- Calculate the angle to position
desired_angle = math.atan(toGoPosY / toGoPosX);
local delta_angle = desired_angle - current_angle;
print("desired angle: " .. desired_angle);
print("current angle: " .. current_angle);
print("delta angle: " .. delta_angle);
-- Set velocity based on angle to turn
if (delta_angle < 0) then
	darwin.setVelocity(0,0,-10);
	print("setting omega to -10");
else
	darwin.setVelocity(0,0,10);
	print("setting omega to 10");
end
end
faceGoalGo = function(hfa)
current_angle = wcm.get_pose().a;
end
faceGoalStop = function(hfa)
end

-- Stop behavior to finish out the code nicely
stopStart = function(hfa)
print("I'm all done")
darwin.stop();
end
stopGo = function(hfa) 
end
stopStop = function(hfa)
end




--###################################################################################################--
--#################################### Behavior Setup ###############################################--
--###################################################################################################--

-- here's where we make the states, a state consist of a name and 3 functions, start, stop, and go
findPosition = makeBehavior("findPosition", findPositionStart, findPositionStop, findPositionGo);
faceDirection = makeBehavior("faceDirection", faceDirectionStart, faceDirectionStop, faceDirectionGo);
walkToPos = makeBehavior("walkToPos", walkToPosStart, walkToPosStop, walkToPosGo);
faceGoal = makeBehavior("faceGoal", faceGoalStart, faceGoalStop, faceGoalGo);
stop = makeBehavior("stop", stopStart, stopStop, stopGo);


--###################################################################################################--
--#################################### Transition Table Setup #######################################--
--###################################################################################################--

-- here's the transition table, it's essentially a dictionary indexed by state, and returns a state
--the state you return will be the next state you pulse
myMachine = makeHFA("myMachine", makeTransition({
	[start] = findPosition, --first thing we do: find our position
	[findPosition] = function()
				if os.clock() - time_start > 5 then
					return faceDirection
				else
					return findPosition
				end
			end,
	[faceDirection] = function()
				local delta_angle = math.abs(desired_angle - current_angle);
				if delta_angle < 0.174 then -- < 10 degrees
					return walkToPos
				else
					return faceDirection
				end
			end,
	[walkToPos] = function()
				local dist = math.sqrt((x_move - wcm.get_pose().x)^2 + (y_move - wcm.get_pose().y)^2);
				local delta_angle = math.abs(desired_angle - current_angle);
				print_count = print_count + 1;
				if print_count > 100 then
					print_count = 0;
					print("distance to goal: " .. dist);
				end
				if dist < 0.05 then -- dist < 0.05 meters
					return faceGoal
				elseif delta_angle > 0.262 then -- > 15 degrees
					return faceDirection
				else
					return walkToPos
				end
			end,
	[faceGoal] = function()
				local delta_angle = math.abs(desired_angle - current_angle);
				if delta_angle < 0.174 then -- < 10 degrees
					return stop
				else
					return faceGoal
				end
			end,
	[stop] = function()
				return stop
			end,
	}),false);

--###################################################################################################--
--#################################### Startup Call #################################################--
--###################################################################################################--

--start "main"
darwin.executeMachine(myMachine);
