-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim_childscriptcall_initialization) then
    file=io.open('d:\\test.txt','w')
    joints_h={-1,-1,-1,-1,-1}
	joints_v={-1,-1,-1,-1}
    
	for i=1,5,1 do
		joints_h[i]=simGetObjectHandle('jointPitch_'..(i))
	end
    for i=1,4,1 do
		joints_v[i]=simGetObjectHandle('jointYaw_'..(i))
	end

	ui=simGetUIHandle('Snake_UI')
	--Initialisation of time
	ti = 0
	ti2 = 0
	tf = 0
	tf2 = 0
	t = 0	
	timeStop = 10
	statRecord = false
	--Time step setting 
	timeStep = 0
    timeStep1 = 0.1 --duration of sending command to the joints
	timeStep2 = 0.1 --timestep of the joint control
	timeStep3 = 0.1
	--Parameter control
	speed_h = math.pi --pitch module speed
	speed_v = math.pi --yaw module speed
	amplitude_h = 0 --pitch module amplitude
	amplitude_v = 0  --yaw module amplitude
	amplitude_vh = 0 --turning amplitude
	k1 = 0  --number of cycle for pitch module 
	k2 = 0	--number of cycle for yaw module
	phase_vh = 0 --phase difference between pitch and yaw modules
	
--Inputs for the function control (in radians) (also 2 sets)
	A_H = amplitude_h*math.pi/180
	A_V = amplitude_v*math.pi/180
	A_VH = amplitude_vh*math.pi/180
	P_H = k1*2/5*math.pi
	P_V = k2*2/4*math.pi
	P_VH = phase_vh*math.pi/180

end


if (sim_call_type==sim_childscriptcall_actuation) then

	buttonID=simGetUIEventButton(ui)
	if (buttonID==3) or (buttonID == 57)then
		timeStep = 1
	end
	if (buttonID==4)then
		timeStep = 0
	end
	if (buttonID==5)then
		timeStep = 0
		ti = 0
		tf = 0
	end
	if (buttonID==40) then
		-- Snake Locomotion Parameter Settings
		speed_h = simGetUIButtonLabel(ui,14)
		speed_h = tonumber(speed_h)
		speed_v = simGetUIButtonLabel(ui,15)
		speed_v = tonumber(speed_v)
		amplitude_h = simGetUIButtonLabel(ui,7)
		amplitude_h = tonumber(amplitude_h)
		A_H = amplitude_h*math.pi/180
		amplitude_v = simGetUIButtonLabel(ui,9)
		amplitude_v = tonumber(amplitude_v)	
		A_V = amplitude_v*math.pi/180
	    amplitude_vh = simGetUIButtonLabel(ui,11)
		amplitude_vh = tonumber(amplitude_vh)
		A_VH = amplitude_vh*math.pi/180
		ph = simGetUIButtonLabel(ui,20)
		ph = tonumber(ph)
		P_H = ph*math.pi/180
		pv = simGetUIButtonLabel(ui,21)
		pv = tonumber(pv)
		P_V = pv*math.pi/180
		phase_vh = simGetUIButtonLabel(ui,17)
		phase_vh = tonumber(phase_vh)
		P_VH = phase_vh*math.pi/180
		-- Timestep settings
		uiTimeStep1 = simGetUIButtonLabel(ui,36)
		uiTimeStep1 = tonumber(uiTimeStep1)
		timeStep1 = uiTimeStep1
		uiTimeStep2 = simGetUIButtonLabel(ui,37)
		uiTimeStep2 = tonumber(uiTimeStep2)
		timeStep2 = uiTimeStep2
		t = simGetUIButtonLabel(ui,58)
		t = tonumber(t)

	end

    --simAddStatusbarMessage(h)
	ti=ti+simGetSimulationTimeStep()
	if (ti - tf) >= timeStep1 then
 
		for i=1,5,1 do 
			simSetJointTargetPosition(joints_h[i],-A_H*math.sin(speed_h*t*timeStep2+(i-1)*P_H))
		end
		for i=1,4,1 do 
			simSetJointTargetPosition(joints_v[i],-A_V*math.sin(speed_v*t*timeStep2+(i-1)*P_V+P_VH)+A_VH)
		end
		tf = ti
		t = t + timeStep
	end
end

if (sim_call_type==sim_childscriptcall_sensing) then

	-- Put your main SENSING code here
	buttonID2 = simGetUIEventButton(ui)
	if (buttonID==57) then
		if (statRecord==false) then
			timeStep3 = simGetUIButtonLabel(ui,49)
			timeStep3 = tonumber(timeStep3)
			timeStop = simGetUIButtonLabel(ui,54)
			timeStop = tonumber(timeStop)
			statRecord = true
			tf2 = 0
			ti2 = 0
			file:write("Star Recording\n")
			posTail = simGetObjectPosition(joints_h[1],-1)
			posMid = simGetObjectPosition(joints_h[3],-1)
			posHead = simGetObjectPosition(joints_h[5],-1)
			angleMid = simGetJointPosition(joints_h[3],-1)
			file:write(ti2..'\t')
			file:write(posTail[1]..'\t')
			file:write(posTail[2]..'\t')
			file:write(posTail[3]..'\t')
			file:write(posMid[1]..'\t')
			file:write(posMid[2]..'\t')
			file:write(posMid[3]..'\t')
			file:write(posHead[1]..'\t')
			file:write(posHead[2]..'\t')
			file:write(posHead[3]..'\t')
			file:write(angleMid..'\n')
		elseif (statRecord==true) then
			statRecord = false
		end
	end
	--h = tostring(statRecord)
	--simAddStatusbarMessage(h)
	if (statRecord == true) then
		ti2=ti2+simGetSimulationTimeStep()
		if ((ti2 - tf2) >= timeStep3 and ti2 < timeStop+0.1) then
			posTail = simGetObjectPosition(joints_h[1],-1)
			posMid = simGetObjectPosition(joints_h[3],-1)
			posHead = simGetObjectPosition(joints_h[5],-1)
			angleMid = simGetJointPosition(joints_h[3],-1)
			file:write(ti2..'\t')
			file:write(posTail[1]..'\t')
			file:write(posTail[2]..'\t')
			file:write(posTail[3]..'\t')
			file:write(posMid[1]..'\t')
			file:write(posMid[2]..'\t')
			file:write(posMid[3]..'\t')
			file:write(posHead[1]..'\t')
			file:write(posHead[2]..'\t')
			file:write(posHead[3]..'\t')
			file:write(angleMid..'\n')
			tf2 = ti2
		elseif (ti2 > timeStop+0.1) then
			statRecord = false
			tf2 = 0
			ti2 = 0
			timeStep = 0
			ti = 0
			tf = 0
		end
	end
end


if (sim_call_type==sim_childscriptcall_cleanup) then

	-- Put some restoration code here
	io.close(file)

end




