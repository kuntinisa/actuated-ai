from __future__ import absolute_import
from __future__ import print_function
from sumolib import checkBinary

import os 
import sys 
import optparse
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import math as math
import subprocess
import itertools


import subprocess
import random
import keras
import h5py
from collections import deque
from keras.layers import Input, Conv2D, Flatten, Dense
from keras.models import Model

#import libraries
import numpy as np
# we need to import some python modules from the $SUMO HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join (os.environ['SUMO_HOME'], 'tools')
    sys.path.append (tools)
else:
    sys.exit("please declare environment variable 'SUMO HOME'")
    
from sumolib import checkBinary # Checks for the binary in environ vars
import traci

fig, ax = plt.subplots()  # Create a figure containing a single axes.
ax.plot([1, 2, 3, 4], [1, 4, 2, 3])  # Plot some data on the axes.



def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

# DEFINE AGENT 
class DQNAgent:
    def __init__(self):
        self.gamma = 0.95   # discount rate
        self.epsilon = 0.1  # exploration rate at first if improved on
        self.epsilon_min = 0.01  # minimum exploration probability
        self.epsilon_decay = 0.0005  # exponential decay rate for exploration prob
        self.epsilon_greedy = False # use epsilon greedy strategy
        self.learning_rate = 0.1 # learning rate dari q learning, kalo 0 berarti q value tidak pernah diupdate
        self.memory = deque(maxlen=50000)
        self.model = self._build_model()
        self.action_size = 4

    def _build_model(self):

        

        input_3 = Input(shape=(4, 1))
        
        x3 = Flatten()(input_3)

        

        input_5 = Input(shape=(4, 1))
       
        x5 = Flatten()(input_5)


        x = keras.layers.concatenate([x3, x5])
        x = Dense(128, activation='relu')(x)
        x = Dense(64, activation='relu')(x)
        x = Dense(4, activation='linear')(x)

        model = Model(inputs=[input_3,  input_5], outputs=[x])
        model.compile(optimizer=keras.optimizers.RMSprop(
            lr=self.learning_rate), loss='mse')

        return model

    def remember(self, state, action, reward, next_state, done):
        print(state)
        log_reinforcement = open('log_reinforcementlearning.txt', 'a')
        log_reinforcement.write(str(action) + ' ' + str(reward) + '\n')
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state, episode):

        if e < 1:
            self.epsilon = 1
        else:
            self.epsilon = 0.1
            
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size), "exploration"
        
        act_values = self.model.predict(state)
        print("(------------------------------------------)")
        log_waitingtime.write("action value"+ str(act_values)+"\n")

        print(np.argmax(act_values[0]))
       
        print("Exploitation mode ", np.argmax(act_values[0]))
        return np.argmax(act_values[0]), "exploitation"
    
    
    # improved epsilon greedy
    # We'll use an improved version of our epsilon greedy strategy for Q-learning, 
    # where we gradually reduce the epsilon as the agent becomes more confident in estimating the Q-values. 
    def act_new(self, state, decay_step):
        # EPSILON GREEDY STRATEGY
        if self.epsilon_greedy:
        # Here we'll use an improved version of our epsilon greedy strategy for Q-learning
            explore_probability = self.epsilon_min + (self.epsilon - self.epsilon_min) * np.exp(-self.epsilon_decay * decay_step)
        # OLD EPSILON STRATEGY
        else:
            if self.epsilon > self.epsilon_min:
                self.epsilon *= (1-self.epsilon_decay)
            explore_probability = self.epsilon

        if explore_probability > np.random.rand():
            # Make a random action (exploration)
            actionz = random.randrange(self.action_size)
            # acc_greedy_exploration += 1
            return actionz, "exploration"
        else:
            # Get action from Q-network (exploitation)
            # Estimate the Qs values state
            # Take the biggest Q value (= the best action)
            act_values = self.model.predict(state)
            actionz = np.argmax(act_values[0])  # returns action
            # acc_greedy_exploitation += 1
            return actionz, "exploitation"
    
    def replay(self, batch_size):
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            # klo done akhir episode, rewardnya ditambah sedikit prediction
            if not done:
                target = (reward + self.gamma *
                          np.amax(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)
            print(target_f)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)

    def load(self, name):
        self.model.load_weights(name)

    def save(self, name):
        self.model.save_weights(name)

# GET STATE IN INTERSECTION
# Class untuk kebutuhan SUMO intersection
class SumoIntersection:
    def get_options(self):
        optParser = optparse.OptionParser()
        optParser.add_option("--nogui", action="store_true",
                             default=False, help="run the commandline version of sumo")
        options, args = optParser.parse_args()
        return options
    
    def getBestGreen(self):
        bestGreenMatrix = []
        vehicle_speed = 13.89
        # mengambil koordinat tengah dari junction
        junctionPosition = traci.junction.getPosition('J3')
        # mengambil semua kendaraan di tiap lane, contoh ('71', '59', '56', '51', '29', '19', '18', '7', '3', '1', '54', '41', '39', '34', '33', '5')
        vehicles_road1 = traci.edge.getLastStepVehicleIDs('-E1')
        vehicles_road2 = traci.edge.getLastStepVehicleIDs('-E2')
        vehicles_road3 = traci.edge.getLastStepVehicleIDs('-E3')
        vehicles_road4 = traci.edge.getLastStepVehicleIDs('-E4')

        # cari kendaraan paling jauh, caranya catat dulu jaraknya dengan junction berapa m
        # jarak yang bisa ditoleransi adalah 583 meter
        farthest_veh = 0
        
        for v in vehicles_road1:
            # jarak antara junction dan kendaraan dikurangi dengan 11 kemudian positifkan lalu dibagi cell length
            # jadi mereka membuat cell dengan lebar 7 
            # offset ini berguna untuk apa?
            jarak = int(
                abs(junctionPosition[1] - traci.vehicle.getPosition(v)[1]))
            if jarak < 583 and farthest_veh < jarak:
                farthest_veh = jarak
        best_green = int(farthest_veh/vehicle_speed)
        bestGreenMatrix.append(best_green)

        farthest_veh = 0
        for v in vehicles_road2:
            # jarak antara junction dan kendaraan dikurangi dengan 11 kemudian positifkan lalu dibagi cell length
            # jadi mereka membuat cell dengan lebar 7 
            # offset ini berguna untuk apa?
            jarak = int(
                abs(junctionPosition[0] - traci.vehicle.getPosition(v)[0]))
            if jarak < 583 and farthest_veh < jarak:
                farthest_veh = jarak
        best_green = int(farthest_veh/vehicle_speed)
        bestGreenMatrix.append(best_green)

        farthest_veh = 0
        for v in vehicles_road3:
            # jarak antara junction dan kendaraan dikurangi dengan 11 kemudian positifkan lalu dibagi cell length
            # jadi mereka membuat cell dengan lebar 7 
            # offset ini berguna untuk apa?
            jarak = int(
                abs(junctionPosition[1] - traci.vehicle.getPosition(v)[1]))
            if jarak < 583 and farthest_veh < jarak:
                farthest_veh = jarak
        best_green = int(farthest_veh/vehicle_speed)
        bestGreenMatrix.append(best_green)

        farthest_veh = 0
        for v in vehicles_road4:
            # jarak antara junction dan kendaraan dikurangi dengan 11 kemudian positifkan lalu dibagi cell length
            # jadi mereka membuat cell dengan lebar 7 
            # offset ini berguna untuk apa?
            jarak = int(
                abs(junctionPosition[0] - traci.vehicle.getPosition(v)[0]))
            if jarak < 583 and farthest_veh < jarak:
                farthest_veh = jarak
        best_green = int(farthest_veh/vehicle_speed)
        bestGreenMatrix.append(best_green)
        # print("best green tiap phase", bestGreenMatrix)
        

        return bestGreenMatrix

    # untuk get data situasi kondisi lingkungan
    def getState(self):
        positionMatrix = []
        velocityMatrix = []
        densityMatrix = []

        cellLength = 7
        offset = 11
        speedLimit = 14

        # mengambil koordinat tengah dari junction
        junctionPosition = traci.junction.getPosition('J3')
        # mengambil semua kendaraan di tiap lane, contoh ('71', '59', '56', '51', '29', '19', '18', '7', '3', '1', '54', '41', '39', '34', '33', '5')
        vehicles_road1 = traci.edge.getLastStepVehicleIDs('-E1')
        vehicles_road2 = traci.edge.getLastStepVehicleIDs('-E2')
        vehicles_road3 = traci.edge.getLastStepVehicleIDs('-E3')
        vehicles_road4 = traci.edge.getLastStepVehicleIDs('-E4')

      

        # buat 12 matrix range
        for i in range(12):
            positionMatrix.append([])
            velocityMatrix.append([])
            for j in range(12):
                positionMatrix[i].append(0)
                velocityMatrix[i].append(0)

       
        # cek tiap kendaraan dalam lane
        for v in vehicles_road1:
            # jarak antara junction dan kendaraan dikurangi dengan 11 kemudian positifkan lalu dibagi cell length
            # jadi mereka membuat cell dengan lebar 7 
            # offset ini berguna untuk apa?
            ind = int(
                abs((junctionPosition[1] - traci.vehicle.getPosition(v)[1] - offset)) / cellLength)
            
            # cell 1-12 yang akan dipakai dimana jarak tiap cell adalah 7, sehingga maksimal 84
            if(ind < 12):
                positionMatrix[2 - traci.vehicle.getLaneIndex(v)][11 - ind] = 1
                velocityMatrix[2 - traci.vehicle.getLaneIndex(
                    v)][11 - ind] = traci.vehicle.getSpeed(v) / speedLimit

        for v in vehicles_road2:
            ind = int(
                abs((junctionPosition[0] - traci.vehicle.getPosition(v)[0] + offset)) / cellLength)
            if(ind < 12):
                positionMatrix[3 + traci.vehicle.getLaneIndex(v)][ind] = 1
                velocityMatrix[3 + traci.vehicle.getLaneIndex(
                    v)][ind] = traci.vehicle.getSpeed(v) / speedLimit

        junctionPosition = traci.junction.getPosition('J3')
        for v in vehicles_road3:
            ind = int(
                abs((junctionPosition[1] - traci.vehicle.getPosition(v)[1] - offset)) / cellLength)
            if(ind < 12):
                positionMatrix[6 + 2 -
                               traci.vehicle.getLaneIndex(v)][11 - ind] = 1
                velocityMatrix[6 + 2 - traci.vehicle.getLaneIndex(
                    v)][11 - ind] = traci.vehicle.getSpeed(v) / speedLimit

        for v in vehicles_road4:
            ind = int(
                abs((junctionPosition[0] - traci.vehicle.getPosition(v)[0] + offset)) / cellLength)
            if(ind < 12):
                positionMatrix[9 + traci.vehicle.getLaneIndex(v)][ind] = 1
                velocityMatrix[9 + traci.vehicle.getLaneIndex(
                    v)][ind] = traci.vehicle.getSpeed(v) / speedLimit

        
        position = np.array(positionMatrix)
        position = position.reshape(1, 12, 12, 1)

        velocity = np.array(velocityMatrix)
        velocity = velocity.reshape(1, 12, 12, 1)

        preempted_phase=[0,0,0,0]

        match preemption_in:
            case "-E1_0"|"-E1_1":
                preempted_phase=[1, 0, 0, 0]
            case "-E2_0"|"-E2_1":
                preempted_phase=[0, 1, 0, 0]
            case "-E3_0"|"-E3_1":
                preempted_phase=[0, 0, 1, 0]
            case "-E4_0"|"-E4_1":
                preempted_phase=[0, 0, 0, 1]

        
        preempted_phase_state = np.array(preempted_phase)
        preempted_phase_state = preempted_phase_state.reshape(1,4,1)

        vehicles_road1 = traci.edge.getLastStepVehicleNumber('-E1')
        vehicles_road2 = traci.edge.getLastStepVehicleNumber('-E2')
        vehicles_road3 = traci.edge.getLastStepVehicleNumber('-E3')
        vehicles_road4 = traci.edge.getLastStepVehicleNumber('-E4')

        light = []
        match traci.trafficlight.getPhase('tls'):
            case 0|1:
                light = [1, 0, 0, 0]
            case 2|3:
                light = [0, 1, 0, 0]
            case 4|5:
                light = [0, 0, 1, 0]
            case 6|7:
                light = [0, 0, 0, 1]
        
        
        densityMatrix = [vehicles_road1, vehicles_road2, vehicles_road3, vehicles_road4]
        
        position = np.array(positionMatrix)
        position = position.reshape(1, 12, 12, 1)

        velocity = np.array(velocityMatrix)
        velocity = velocity.reshape(1, 12, 12, 1)

        density = np.array(densityMatrix)
        density = density.reshape(1,4,1)

        print(preempted_phase)
        print(density)


        lgts = np.array(light)
        lgts = lgts.reshape(1, 4, 1)

        return [density, preempted_phase_state]
    
    def getReward(self):
       
        vehicles_road1 = (traci.lane.getLastStepHaltingNumber('-E1_0')+traci.lane.getLastStepHaltingNumber('-E1_1'))
        vehicles_road2 = (traci.lane.getLastStepHaltingNumber('-E2_0')+traci.lane.getLastStepHaltingNumber('-E2_1'))
        vehicles_road3 = (traci.lane.getLastStepHaltingNumber('-E3_0')+traci.lane.getLastStepHaltingNumber('-E3_1'))
        vehicles_road4 = (traci.lane.getLastStepHaltingNumber('-E4_0')+traci.lane.getLastStepHaltingNumber('-E4_1'))
        waitingtimeMatrix = [vehicles_road1, vehicles_road2, vehicles_road3, vehicles_road4]
        
        return waitingtimeMatrix


# TRAIN THE MODEL IN THE LOOP
# contains TraCI control loop

# def run ():
    
    # traci.close()
    # sys.stdout.flush()
    
    # main entry point
   
if __name__ == '__main__':
    sumoInt = SumoIntersection()
    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    options = sumoInt.get_options()

    if options.nogui:
    #if True:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # Main logic
    # parameters
    episodes = 2000
    batch_size = 100
    dtt_array=[]
    


    tg = 10
    ty = 6
    agent = DQNAgent()
    try:
        agent.load('Models/reinf_traf_control.h5')
    except:
        print('No models found')

    for e in range(episodes):
        dtt_count=0
        dtt_phase = []
        log_reward = open('log_reward.txt', 'a')

        acc_greedy_exploration = 0
        acc_greedy_exploitation = 0
        log_greedy = open('log_greedy.txt', 'a')

        acc_waiting_time=0
        waiting_time=0
        log_waitingtime = open('log_waitingtime.txt', 'a')
        log_waitedtime = open('log_waitedtime.txt', 'a')
        log_preemptiontimer = open('log_preemptiontimer.txt', 'a')
        log_episode = open('log_episode.txt', 'a')

        step = 0
        preemption_timer_stop_at = 0
        # contoh 1 persimpangan variabel
        number_of_intersection = 1;
        persimpangans=['gramedia', 'tugu', 'pingit']
        phases = ['timur', 'barat']

        # preemption variable
        # status preemption saat ini ('', on, done)
        preemption_status=''
        # traffic light phase yang sedang berjalan saat ini (0,1,2,3,4,5,6,7)
        current_trafficlight=''
        # preemption sedang berlangsung selama berapa detik
        preemption_timer=0
        # preemption keberapa sekarang
        preemption_count=0
        # ada berapa vehicle dalam preemption yang sedang berlangsung? biasanya ada 1
        vehicle_on_preemption=[]
        # preemption berlangsung selama berapa detik / ketika preemption berhenti, di detik keberapa
        preemption_timer_stop_at=0

        # exit strategy variable
        # ini hanya untuk satu intersection
        phase_status=''
        exitstrategy_start_at_timestep=''


        es3_enddwell_status=''
        enddwell_start_at_timestep=0
        all_duration = 45
        green_duration = 25
        yellow_duration = 3 



        # 2. fixed phase exit strategy
        fixedphase_trafficlight=""
        fixedphase_trafficlight_duration=""
        
        # 3. coordpreempt variable 
        coord_preempt_trafficlight=""
        coord_preempt_trafficlight_duration=""

        # 4. end dwell variable
        enddwell_phase=[]

        # 5. dynamic threshold timer
        waitedtime_phase=[]
        thresholdtimer=0
        thresholdtimer_status=""
        threshold_start_at_timestep=''

        preemption_on_step=0
        preemption_in=''
        green_wave=''


        # useless variabel
        none=""

        max_waitingline_phase=none;
        
        tls_cycle_times = [25,5,25,5,25,5,25,5]
        green_duration=25
        yellow_duration=5
        phase_duration=green_duration+yellow_duration
        red_duration=90
        tls_cycles_part = [0,1,2,3,4,5,6,7]
        phase=[[0,1], [2,3], [4,5], [6,7]]
        ev_count = 0
        cum_preemption=0
        cum_reward=0
        acc_cum_reward=0
        cum_negativereward=0

        count_es1=0
        count_es2=0
        count_es3=0
        count_es4=0
        all_es=[]

        vehiclenumbers = ""
        current_trafficlightelapsed =""
        

        chosen_exit_strategy = 0
        threshold_start_at_timestepexit=0
        awt = []
        prev_action=''


        in_detector=["-E1_0", "-E1_1", "-E2_0", "-E2_1", "-E3_0", "-E3_1", "-E4_0", "-E4_1"]
        out_detector=["E1_0", "E1_1", "E2_0", "E2_1", "E3_0", "E3_1", "E4_0", "E4_1", ]

        # membuat random ev
        range_period = [900, 600, 450]
        period = random.choice([300])
     
        def rand_float_range(start, end):
            return random.random() * (end - start) + start
        # round(number, decimal_point)
        # period_nonev = round(rand_float_range(1.8, 9), 1)
        period_nonev = random.randint(2, 9)
        period_nonev = 2
        # print("python3 ../randomTrips.py -n lefthand.net.xml -r four-leg-thesis-intersection.rou.xml -b 0 -e 1800 --period "+ str(period_nonev) +" --fringe-factor 10 --random")
        # os.system("python3 ../randomTrips.py -n lefthand.net.xml -r four-leg-thesis-intersection.rou.xml -b 0 -e 3600 --period "+ str(period_nonev) +" --fringe-factor 10 --random")
        # exit()
        traci.start([sumoBinary, "-c", "osm_0.sumocfg", '--start', '--quit-on-end'])
        

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
            print("step ", step)
            # tiap step berjalan catat waiting timenya
            waiting_time += (traci.lane.getLastStepHaltingNumber('-E1_0')
                             +traci.lane.getLastStepHaltingNumber('-E1_1')
                             +traci.lane.getLastStepHaltingNumber('-E2_0')
                             +traci.lane.getLastStepHaltingNumber('-E2_1')
                             +traci.lane.getLastStepHaltingNumber('-E3_0')
                             +traci.lane.getLastStepHaltingNumber('-E3_1')
                             +traci.lane.getLastStepHaltingNumber('-E4_0')
                             +traci.lane.getLastStepHaltingNumber('-E4_1')
                             )
            
            acc_waiting_time += waiting_time

            # deteksi EV masuk dengan inductionloop detector
            for evdetector in in_detector:
                # jika EV terdeteksi oleh induction loop
                if traci.inductionloop.getLastStepVehicleNumber(evdetector) > 0:
                    # jalankan preemption saat tidak ada preemption lain
                    preemption_in=evdetector
                    if preemption_status!="on":
                        # harusnya disini simpan dulu reward dari action yang sedang berlangsung
                        # tapi kalo hanya berjalan sebentar mending gausa disimpan daripada mengaburkan q-table
                        # agent.remember(state, action, cum_reward, new_state, False)
                        # acc_cum_reward += cum_reward
                        # print("\nStore reward ", cum_reward, "into memory")
                        # cum_reward=0
                        # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
                        if(len(agent.memory) > batch_size):
                            agent.replay(batch_size)
                        phase_status=""
                        print("GANTI PHASE ===============================")
                        # get phase tls saat EV terdeteksi (0,1,2,3,4,5,6,7)
                        current_trafficlight = traci.trafficlight.getPhase("tls")
                        # get duration tls saat EV terdeteksi (second)
                        current_trafficlightduration = traci.trafficlight.getPhaseDuration("tls")
                        # get duration tersisa tls saat EV terdeteksi
                        current_trafficlightelapsed = traci.trafficlight.getNextSwitch("tls")-traci.simulation.getTime()
                        # menentukan jalur / phase jalanan tls saat EV terdeteksi (0,1,2,3)
                        get_phase = [0,0,1,1,2,2,3,3]
                        now_phase = get_phase[current_trafficlight]
                        phase_cycle = [0,1,2,3]
                        
                        # get duration yang telah dijalankan tls saat EV terdeteksi
                        # hitung tls sekarang udah jalan berapa detik
                        if current_trafficlight%2 == 0:
                            tls_done_duration = green_duration-current_trafficlightelapsed
                        else:
                            tls_done_duration = yellow_duration-current_trafficlightelapsed
                       
                        print("\n \n====================================================")
                        print("EV detected in by ", evdetector, " in step ", step)
                        print("Current signal phase interupted ", current_trafficlight, " in road ",now_phase," remaining time ", current_trafficlightelapsed, " s, has elapsed in ", tls_done_duration, " s")
                        
                        
                        # kalkulasi waktu tunggu jalur lain non preempt
                        phase=[[0,1], [2,3], [4,5], [6,7]]
                        waitedtime_phase=[0,0,0,0]
                        waitedtime_phase[now_phase] = 0
                        # next array
                        now_phase = (now_phase + 1) % len(phase_cycle)
                        waitedtime_phase[now_phase] = red_duration-phase_duration*1+tls_done_duration
                        # next array
                        now_phase = (now_phase + 1) % len(phase_cycle)
                        waitedtime_phase[now_phase] = red_duration-phase_duration*2+tls_done_duration
                        # next array
                        now_phase = (now_phase + 1) % len(phase_cycle)
                        waitedtime_phase[now_phase] = red_duration-phase_duration*3+tls_done_duration
                        # karna preemption berjalan di phase 0 maka 0 kan phase ini
                        waitedtime_phase[0] = 0
                        print("non preempt phase waited time: ", waitedtime_phase)
                        
                        # set tls ke yellow untuk phase saat EV terdeteksi 
                        # if 0,2,4,6 maka jadikan kuning dulu 0->1, 2->3 selama 6 detik
                        match traci.trafficlight.getPhase("tls"):
                            case 0:
                                traci.trafficlight.setPhase("tls", 1)
                            case 2:
                                traci.trafficlight.setPhase("tls", 3)
                            case 4:
                                traci.trafficlight.setPhase("tls", 5)
                            case 6:
                                traci.trafficlight.setPhase("tls", 7)
                        traci.trafficlight.setPhaseDuration("tls", yellow_duration)

                                
                        # kebutuhan ES coord_preempt menyelaraskan tbc dan loc
                        coord_preempt_trafficlight_duration = current_trafficlightelapsed
                        coord_preempt_trafficlight = current_trafficlight

                        # jika ada EV yang terdeteksi maka ubah status preemption ke on 
                        if traci.inductionloop.getLastStepVehicleIDs(evdetector) != "()":
                            vehicle_on_preemption.append(traci.inductionloop.getLastStepVehicleIDs(evdetector))
                            preemption_status="on"
                            ev_count =+ 1
                            match preemption_in:
                                case "-E1_0"|"-E1_1":
                                    green_wave=0
                                case "-E2_0"|"-E2_1":
                                    green_wave=2
                                case "-E3_0"|"-E3_1":
                                    green_wave=4
                                case "-E4_0"|"-E4_1":
                                    green_wave=6
                            dtt_phase.clear()
                            print("\nGreen wave for Emergency Vehicle Start for: ", vehicle_on_preemption)
            # selesai mendeteksi di semua detector masuk

            # preemption sedang berlangsung            
            if preemption_status=="on":
                print("W", end = '')
                
                # green tls jalur preemption selama status preemption on
                # preemption green tls dimulai pertama kali
                traci.trafficlight.setPhase("tls", green_wave)
                traci.trafficlight.setPhaseDuration("tls", 1)
                
                # hitung lamanya preemption berlangsung
                preemption_timer += 1

                # ES coord_preempt kurangi duration untuk menyelaraskan tbc dan loc
                coord_preempt_trafficlight_duration -= 1 
                if coord_preempt_trafficlight_duration==0:
                    # next array
                    next = (coord_preempt_trafficlight + 1) % len(tls_cycles_part)
                    # print("phase selanjutnya : ", next)
                    coord_preempt_trafficlight = next
                    coord_preempt_trafficlight_duration=tls_cycle_times[next]
                
            # deteksi EV keluar dengan induction loop
            for evdetector in out_detector:
                # jika ada EV yang terdeteksi keluar
                if traci.inductionloop.getLastStepVehicleNumber(evdetector):
                    # jika EV tersebut benar sama dengan yang dideteksi oleh induction loop sebelumnya
                    if traci.inductionloop.getLastStepVehicleIDs(evdetector) in vehicle_on_preemption:
                        print("\nEV detected out by ", evdetector, "pada detik ke ", step)
                        # remove EV dari daftar EV yang sedang preemption
                        vehicle_on_preemption.remove(traci.inductionloop.getLastStepVehicleIDs(evdetector))
                        # Jika EV dalam daftar sudah habis
                        if not vehicle_on_preemption:
                            # catat lama preemption
                            preemption_timer_stop_at = preemption_timer
                            # ubah status preemption
                            preemption_status="done"
                            phase_status==''
                            # yellow tls selama 6 detik sebelum ke ES
                            match preemption_in:
                                case "-E1_0"|"-E1_1":
                                    traci.trafficlight.setPhase("tls", 1)
                                case "-E2_0"|"-E2_1":
                                    traci.trafficlight.setPhase("tls", 3)
                                case "-E3_0"|"-E3_1":
                                    traci.trafficlight.setPhase("tls", 5)
                                case "-E4_0"|"-E4_1":
                                    traci.trafficlight.setPhase("tls", 7)
                            traci.trafficlight.setPhaseDuration("tls", yellow_duration)


            # jika semua ev telah melintas, matikan timer preemption 
            if not vehicle_on_preemption:
                preemption_timer = 0;
            
            # jika preemption selesai, pilih exit strategy
            if preemption_status=="done" :
                # hitung ini preemption ke berapa
                preemption_count+=1
                print("Preemption [", preemption_count, "] ", preemption_status, " in ", preemption_timer_stop_at, " s \n")
                phase_status = ""
                prev_action = ''
               
                cum_preemption = cum_preemption + preemption_timer_stop_at
                log_preemptiontimer.write(str(preemption_timer_stop_at) + '\n')

                # get waiting line non EV di semua jalur ketika preemption selesai
                vehiclenumbers=[
                    int(traci.lane.getLastStepVehicleNumber("-E1_0")+
                    traci.lane.getLastStepVehicleNumber("-E1_1")), 
                    
                    int(traci.lane.getLastStepVehicleNumber("-E2_0")+ 
                    traci.lane.getLastStepVehicleNumber("-E2_1")),

                    int(traci.lane.getLastStepVehicleNumber("-E3_0")+ 
                    traci.lane.getLastStepVehicleNumber("-E3_1")),

                    int(traci.lane.getLastStepVehicleNumber("-E4_0")+ 
                    traci.lane.getLastStepVehicleNumber("-E4_1"))
                    ]
                
                
                
            if preemption_timer!=0:
                none=""
            else: 
                preemption_status=""

            if preemption_status!="on" and phase_status=='':
                state = sumoInt.getState()
                action = agent.act(state, e)[0]
                bestgreen = sumoInt.getBestGreen()

                print("best green saat action diambil : ", bestgreen)
                print("action dipilih ", action)
                phase_status="on"
                awt = sumoInt.getReward()
                exitstrategy_start_at_timestep = step
                print("phase status ", phase_status)

                match action:
                    case 0|1|2|3:
                        # 2. fixed phase exit strategy 
                        print("Vehicle density : ", vehiclenumbers)
                        action_phase = ['',0,1,2,3,'',0,1,2,3,0,1,2,3]
                        action_phase = [0,1,2,3]
                        max_waitingline_phase = action_phase[chosen_exit_strategy]
                        print("Programmed exit phase ", max_waitingline_phase, "for : ", all_duration," s")
                        print("Run interrupted signal phase ", current_trafficlight, "for about ", current_trafficlightelapsed, "s")

                
            if preemption_status!="on" and phase_status=="on":
                if bestgreen[action]==0:
                    # reward = 0
                    # cum_reward = cum_reward+reward
                    # new_state= sumoInt.getState()
                    # agent.remember(state, action, cum_reward, new_state, False)
                    # acc_cum_reward += cum_reward
                    # print("\nStore reward ", cum_reward, "into memory")
                    # cum_reward=0
                    # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
                    # if(len(agent.memory) > batch_size):
                        # agent.replay(batch_size)
                    phase_status=""
                    print("GANTI PHASE ===============================")
                else:
                    # transition berlaku jika action sebelumnya berbeda dengan action saat ini
                    if prev_action!='' and action!=prev_action: 
                        if (exitstrategy_start_at_timestep <= step <= int(exitstrategy_start_at_timestep+3)):
                            traci.trafficlight.setPhase("tls", phase[prev_action][0]+1)
                            traci.trafficlight.setPhaseDuration("tls", 1)
                        if (exitstrategy_start_at_timestep+3 <= step <= int(exitstrategy_start_at_timestep+3+bestgreen[action])):
                            traci.trafficlight.setPhase("tls", phase[action][0])
                            traci.trafficlight.setPhaseDuration("tls", 1)
                            print("phase sedang berlangsung di tls", phase[action][0])
                            # cek reward 
                            now_awt = sumoInt.getReward()
                            reward = sum(np.subtract(awt,now_awt))
                            awt=now_awt
                            # negative waiting time
                            # reward=-sum(now_awt)
                            print("reward step ini: ", reward)
                            cum_reward = cum_reward+reward
                        if (step == int(exitstrategy_start_at_timestep+3+bestgreen[action])):
                            new_state= sumoInt.getState()
                            agent.remember(state, action, cum_reward, new_state, False)
                            acc_cum_reward += cum_reward
                            print("\nStore reward ", cum_reward, "into memory")
                            cum_reward=0
                            # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
                            if(len(agent.memory) > batch_size):
                                agent.replay(batch_size)
                            phase_status=""
                            prev_action = action
                            print("GANTI PHASE ===============================")
                    else:
                        if (exitstrategy_start_at_timestep <= step <= int(exitstrategy_start_at_timestep+bestgreen[action])):
                            traci.trafficlight.setPhase("tls", phase[action][0])
                            traci.trafficlight.setPhaseDuration("tls", 1)
                            print("phase sedang berlangsung di tls", phase[action][0])
                            # cek reward 
                            now_awt = sumoInt.getReward()
                            reward = sum(np.subtract(awt,now_awt))
                            
                            awt=now_awt
                            # reward=-sum(now_awt)
                            print("reward step ini: ", reward)
                            cum_reward = cum_reward+reward

                        if (step == int(exitstrategy_start_at_timestep+bestgreen[action])):
                            new_state= sumoInt.getState()
                            agent.remember(state, action, cum_reward, new_state, False)
                            acc_cum_reward += cum_reward
                            print("\nStore reward ", cum_reward, "into memory")
                            cum_reward=0
                            # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
                            if(len(agent.memory) > batch_size):
                                agent.replay(batch_size)
                            phase_status=""
                            prev_action = action
                            print("GANTI PHASE ===============================")
                    
            step += 1

           
            
        # normal vehicle count, ev count, cumulative preemption duration, average waiting time,
        
        if len(agent.memory) > 0:
            mem = agent.memory[-1]
            del agent.memory[-1]
            agent.memory.append((mem[0], mem[1], reward, mem[3], True))
        nev_count = math.ceil(1800/period_nonev)
        nev_count = 1800
        avg_waitingtime = round(acc_waiting_time/(nev_count+ev_count), 2)
        avg_preemptionduration=0
        if (cum_preemption!=0):
            avg_preemptionduration = round(cum_preemption/(ev_count), 0) 
        log_episode.write(str(chosen_exit_strategy)+" "+str(max_waitingline_phase)+" ")
        log_episode.write(str(nev_count) + " " + str(preemption_count) + " " + str(avg_preemptionduration) + " " + str(all_es) + " " + str(count_es1)+ " " + str(count_es2)+ " " + str(count_es3)+ " " + str(count_es4) + " " + str(acc_cum_reward) + " " + str(cum_negativereward) + " " + str(avg_waitingtime) + " " + '\n')
        acc_cum_reward=0
        all_es = []
        cum_negativereward=0
        # log_waitingtime.write(str(acc_waiting_time) + ' ' + str(dtt_count) + ' \n')
     
        print("\nEnd this episode in step ", step, "\n \n")
        print('episode - ' + str(e))
        traci.close(wait=False)

sys.stdout.flush()