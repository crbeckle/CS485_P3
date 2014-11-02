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
require "math"  -- Used for trigonometric functions
delta_angle = 0.0
distance = 0.0

--###################################################################################################--
--#################################### Behavior Definitions #########################################--
--###################################################################################################--



findBallStart = function(hfa)
print("trying to locate the ball")
darwin.scan();
darwin.setVelocity(0,0,10);
end
findBallGo = function(hfa)
end
findBallStop = function(hfa)
-- Found the ball at this point, so track it and stop turning
darwin.setVelocity(0,0,0);
darwin.stop();
darwin.track();
end

moveToBallStart = function(hfa)
print("moving towards the ball")
print("ball x: " .. wcm.get_ball_x())
print("ball y: " .. wcm.get_ball_y())
delta_angle = math.atan(wcm.get_ball_y() / wcm.get_ball_x());
distance = math.sqrt(wcm.get_ball_x()^2 + wcm.get_ball_y()^2);
print("ball dist: " .. distance)
print("ball a: " .. delta_angle)
darwin.setVelocity((0.25*math.log((1+distance),10)),0,delta_angle);
end
moveToBallGo = function(hfa)
delta_angle = math.atan(wcm.get_ball_y() / wcm.get_ball_x());
distance = math.sqrt(wcm.get_ball_x()^2 + wcm.get_ball_y()^2);
darwin.setVelocity((0.25*math.log((1+distance),10)),0,delta_angle);
end
moveToBallStop = function(hfa)
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
findBall = makeBehavior("findBall", findBallStart, findBallStop, findBallGo);
moveToBall = makeBehavior("moveToBall", moveToBallStart, moveToBallStop, moveToBallGo);
stop = makeBehavior("stop", stopStart, stopStop, stopGo);


--###################################################################################################--
--#################################### Transition Table Setup #######################################--
--###################################################################################################--

-- here's the transition table, it's essentially a dictionary indexed by state, and returns a state
--the state you return will be the next state you pulse
goToBall = makeHFA("goToBall", makeTransition({
	[start] = findBall, --first thing we do: find the ball
	[findBall] = function()
				if vcm.get_ball_detect() then
					return moveToBall
				else
					return findBall
				end
			end,
	[moveToBall] = function()
				if not vcm.get_ball_detect() then
					return findBall
				elseif distance < 0.05 then
					return stop
				else
					return moveToBall
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
darwin.executeMachine(goToBall);
