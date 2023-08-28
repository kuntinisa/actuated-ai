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
        # self.learning_rate = 0.0002 # learning rate dari q learning, kalo 0 berarti q value tidak pernah diupdate
        self.learning_rate = 0.9 # learning rate dari q learning, kalo 0 berarti q value tidak pernah diupdate
        self.memory = deque(maxlen=200)
        self.model = self._build_model()
        # action mode exit strategy ada 5
        self.action_size = 5

    def _build_model(self):

        input_1 = Input(shape=(4, 1))
        x1 = Flatten()(input_1)

        input_2 = Input(shape=(1))
        x2 = Flatten()(input_2)

        input_3 = Input(shape=(2, 1))
        x3 = Flatten()(input_3)

        x = keras.layers.concatenate([x1, x2, x3])
        x = Dense(128, activation='relu')(x)
        x = Dense(64, activation='relu')(x)
        x = Dense(5, activation='linear')(x)

        model = Model(inputs=[input_1, input_2, input_3], outputs=[x])
        model.compile(optimizer=keras.optimizers.RMSprop(
            lr=self.learning_rate), loss='mse')

        return model

    def remember(self, state, action, reward, next_state, done):
        log_reinforcement = open('log_reinforcementlearning.txt', 'a')
        log_reinforcement.write(str(action) + ' ' + str(reward) + '\n')
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state, episode):

        if e < 75:
            self.epsilon = 1
        else:
            self.epsilon = 0.1
            
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size), "exploration"
        
        act_values = self.model.predict(state)
        print("(------------------------------------------)")
        print(act_values)
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
            if not done:
                target = (reward + self.gamma *
                          np.amax(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)
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
    
    # untuk get data situasi kondisi lingkungan
    def getState(self):
        densityMatrix = []
        preemption_durationMatrix = []
        tls_phaseMatrix = []
       
        # get density = jumlah kendaraan di tiap jalur
        vehicles_road1 = len(traci.edge.getLastStepVehicleIDs('-E1'))
        vehicles_road2 = len(traci.edge.getLastStepVehicleIDs('-E2'))
        vehicles_road3 = len(traci.edge.getLastStepVehicleIDs('-E3'))
        vehicles_road4 = len(traci.edge.getLastStepVehicleIDs('-E4'))
        # REVISI get density = waiting time reduction
        # vehicles_road1 = len(traci.lane.getLastStepVehicleIDs('-E1_0')+traci.lane.getLastStepVehicleIDs('-E1_1'))
        # vehicles_road2 = len(traci.lane.getLastStepVehicleIDs('-E2_0')+traci.lane.getLastStepVehicleIDs('-E2_1'))
        # vehicles_road3 = len(traci.lane.getLastStepVehicleIDs('-E3_0')+traci.lane.getLastStepVehicleIDs('-E3_1'))
        # vehicles_road4 = len(traci.lane.getLastStepVehicleIDs('-E4_0')+traci.lane.getLastStepVehicleIDs('-E4_1'))
        densityMatrix = [vehicles_road1, vehicles_road2, vehicles_road3, vehicles_road4]
        
        
        density_state = np.array(densityMatrix)
        density_state = density_state.reshape(1,4,1)

        # preemption_duration = preemption_timer_stop_at
        preemption_durationMatrix = [preemption_timer_stop_at]
        preemption_duration_state = np.array(preemption_durationMatrix)
        preemption_duration_state = preemption_duration_state.reshape(1)


        # skrg di phase apa 0,1,2,3 sudah berjalan berapa lama 0-30 detik

        tls_phaseMatrix = [traci.trafficlight.getPhase("tls"), traci.trafficlight.getPhaseDuration("tls")]
        tls_phase_state = np.array(tls_phaseMatrix)
        tls_phase_state = tls_phase_state.reshape(1,2,1)

        # kepadatan jalan 
        # durasi preemption
        # phase traffic light traci.trafficlight.getPhase('0')
        # print(density_state, preemption_duration_state, tls_phase_state)
        return [density_state, preemption_duration_state, tls_phase_state]


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
    batch_size = 1024
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

        waiting_time=0
        log_waitingtime = open('log_wsaitingtime.txt', 'a')
        log_waitedtime = open('log_waitedtime.txt', 'a')
        log_preemptiontimer = open('log_preemptiontimer.txt', 'a')
        log_episode = open('log_epsiode.txt', 'a')

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
        exitstrategy_status=''
        exitstrategy_start_at_timestep=''


        es3_enddwell_status=''
        enddwell_start_at_timestep=0
        all_duration = 45
        green_duration = 42
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
        
        tls_cycle_times = [42,3,42,3,42,3,42,3]
        green_duration=42
        yellow_duration=3
        phase_duration=green_duration+yellow_duration
        red_duration=135
        tls_cycles_part = [0,1,2,3,4,5,6,7]
        phase=[[0,1], [2,3], [4,5], [6,7]]
        ev_count = 0
        cum_preemption=0
        cum_reward=0
        cum_negativereward=0

        count_es1=0
        count_es2=0
        count_es3=0
        count_es4=0
        count_es5=0


        in_detector=["-E1_0", "-E1_1", "-E2_0", "-E2_1", "-E3_0", "-E3_1", "-E4_0", "-E4_1"]
        out_detector=["E1_0", "E1_1", "E2_0", "E2_1", "E3_0", "E3_1", "E4_0", "E4_1", ]


        # osm_file = "osm_"+str(e)+".sumocfg"
        # print(osm_file)
        # exit()


        # if (e==0):
        
        # traci.start([sumoBinary, "-c", "osm_"+str(e)+".sumocfg", '--start'])
        # membuat random ev
        range_period = [450, 600, 900]
        period = random.choice(range_period)
        # print("python3 ../randomTrips.py -n four-leg-intersection.net.xml -r evfour-leg-new-intersection.rou.xml -b 0 -e 1800 --vehicle-class emergency --vclass emergency --period "+ str(period) +" --random-depart --fringe-factor 10 --random --prefix ev")
        os.system("python3 ../randomTrips.py -n four-leg-intersection.net.xml -r evfour-leg-new-intersection.rou.xml -b 0 -e 1800 --vehicle-class emergency --vclass emergency --period "+ str(period) +" --random-depart --fringe-factor 10 --random --prefix ev")
        # membuat random traffic
        # exit()
        
        # pilih random jumlah kendaraan antara 200-1000
        # 1800/x = 200, x=9
        # 1800/x = 1000 x=1,8
        def rand_float_range(start, end):
            return random.random() * (end - start) + start
        # round(number, decimal_point)
        period_nonev = round(rand_float_range(1.8, 9), 1)
        # print("python3 ../randomTrips.py -n lefthand.net.xml -r four-leg-new-intersection.rou.xml -b 0 -e 1800 --period "+ str(period_nonev) +" --fringe-factor 10 --random")
        os.system("python3 ../randomTrips.py -n lefthand.net.xml -r four-leg-new-intersection.rou.xml -b 0 -e 1800 --period "+ str(period_nonev) +" --fringe-factor 10 --random")
        # exit()
        traci.start([sumoBinary, "-c", "osm_0.sumocfg", '--start'])
        

        while traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()

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

            # disini perulangan di tiap intersection
            
            # deteksi EV masuk dengan inductionloop detector
            for evdetector in in_detector:
                # jika EV terdeteksi oleh induction loop
                if traci.inductionloop.getLastStepVehicleNumber(evdetector) > 0:
                    # jalankan preemption saat tidak ada preemption lain
                    preemption_in=evdetector
                    if preemption_status!="on":
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
                # get waiting line terpanjang ada di jalur mana
                max_waitingline_phase=vehiclenumbers.index(max(vehiclenumbers))

                print("AI Exit Strategy start")
                exit_strategy=['1. No Exit Phase', 
                               '2. Fixed Phase (dynamic)', 
                               '3. Coord Preempt', 
                               '4. End Dwell', 
                               '5. Dynamic Threshold Timer']
                state = sumoInt.getState()
                action = agent.act(state, e)[0]
               
                
                waiting_time_all_phase_before = (traci.lane.getLastStepHaltingNumber('-E1_0')
                             +traci.lane.getLastStepHaltingNumber('-E1_1')
                             +traci.lane.getLastStepHaltingNumber('-E2_0')
                             +traci.lane.getLastStepHaltingNumber('-E2_1')
                             +traci.lane.getLastStepHaltingNumber('-E3_0')
                             +traci.lane.getLastStepHaltingNumber('-E3_1')
                             +traci.lane.getLastStepHaltingNumber('-E4_0')
                             +traci.lane.getLastStepHaltingNumber('-E4_1')
                             )

                # if agent.act(state, e)[1] == 'exploration':
                #     acc_greedy_exploration += 1
                # else:
                #     acc_greedy_exploitation += 1


                #  return [position, velocity, lgts]
                light = state[2]
                
                chosen_exit_strategy = action
                # chosen_exit_strategy = 1
                print("Action chosen : ", exit_strategy[chosen_exit_strategy])
                
                # jalankan exit strategy yang telah dipilih
                exitstrategy_status = "on"
                exitstrategy_start_at_timestep = step



                match chosen_exit_strategy:
                    case 0:
                        print("Run interrupted signal phase ", current_trafficlight, "for about ", current_trafficlightelapsed, "s")
                        count_es1 += 1
                    case 1:
                        # 2. fixed phase exit strategy 
                        count_es2 += 1
                        print("Vehicle density : ", vehiclenumbers)
                        print("Programmed exit phase ", max_waitingline_phase, "for : ", all_duration," s")
                        print("Run interrupted signal phase ", current_trafficlight, "for about ", current_trafficlightelapsed, "s")
                        

                    case 2:
                        # 3. end dwell menghitung lama preemption akan berhenti ketika phase apa
                        # catat fase apa yang diinterupsi 
                        # current_trafficlight = current_trafficlight
                        # current_trafficlightduration = current_trafficlightduration
                        # hitung berapa lama preemption terjadi
                        # preemption_timer_stop_at = preemption_timer_stop_at
                        # hitung seharusnya sekarang phase apa berapa lama (menyamakan loc dan tbc)
                        # 0-24, 6
                        # 1-24, 6
                        # 2-24, 6
                        # 3-24, 6
                        count_es3 += 1
                        print("Programmed exit phase", coord_preempt_trafficlight, "for ", coord_preempt_trafficlight_duration, " s")
                        # traci.trafficlight.setPhase("tls", coord_preempt_trafficlight)
                        # traci.trafficlight.setPhaseDuration("tls", coord_preempt_trafficlight_duration)
                        # print("ga perlu kembali ke cycle awal ", current_trafficlight, "selama : ", current_trafficlightelapsed, "detik, tapi lanjutkan karna tbc dan loc udah sesuai")
                        enddwell_start_at_timestep=step
                        
                        # kembali ke tls yang terhenti karna preemption
                        # traci.trafficlight.setPhase("tls", current_trafficlight)
                        # traci.trafficlight.setPhaseDuration("tls", current_trafficlightelapsed)

                    case 3:
                        count_es4 += 1
                        exitstrategy_status = "on"
                        print("Vehicle density : ", vehiclenumbers)
                        
                        # get urutan non preempt phase dari tertinggi ke terendah
                        maxList = vehiclenumbers

                        temp = ([i[0] for i in sorted(enumerate(maxList), key=lambda k: k[1], reverse=True)])
                        # hilangkan phase 0
                        
                        # Remove preempted phase
                        match preemption_in:
                                case "-E1_0"|"-E1_1":
                                    temp.remove(0)
                                case "-E2_0"|"-E2_1":
                                    temp.remove(1)
                                case "-E3_0"|"-E3_1":
                                    temp.remove(2)
                                case "-E4_0"|"-E4_1":
                                    temp.remove(3)
                        enddwell_phase = temp
                        print(temp)
                        print("Coord+preempt will start at step : ", step)
                        enddwell_start_at_timestep=step
                        

                    case 4:
                        # cek non preempt phase menunggu berapa lama
                        # tetapkan threshold misal 90+10 detik
                        count_es5 += 1
                        # ini yang dibuat AI milih sendiri threshold timer berapa
                        dtt_array = [10, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230, 240, 250, 260, 270, 280, 290, 300, 310, 320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 420, 430, 440, 450, 460, 470, 480, 490, 500, 510, 520, 530, 540, 550, 560, 570, 580, 590, 600, 610, 620, 630, 640, 650, 660, 670, 680, 690, 700, 710, 720, 730, 740, 750, 760, 770, 780, 790, 800, 810, 820, 830, 840, 850, 860, 870, 880, 890, 900, 910, 920, 930, 940, 950, 960, 970, 980, 990]
                        thresholdtimer = 10
                        # print("Threshold value ", dtt_array[e], "s")
                        # exit()


                        for idx, waited_phase in enumerate(waitedtime_phase) :
                            waitedtime_phase[idx] = waited_phase+preemption_timer_stop_at
                        print("Vehicle density :", waitedtime_phase)

                        
                        # while max(waitedtime_phase) >= thresholdtimer:
                        if max(waitedtime_phase) >= thresholdtimer:
                            exitstrategy_status = "on"
                            # nolkan preempted phase
                            match preemption_in:
                                case "-E1_0"|"-E1_1":
                                    waitedtime_phase[0] = 0;
                                case "-E2_0"|"-E2_1":
                                    waitedtime_phase[1] = 0;
                                case "-E3_0"|"-E3_1":
                                    waitedtime_phase[2] = 0;
                                case "-E4_0"|"-E4_1":
                                    waitedtime_phase[3] = 0;
                            print("Reset preempted phase ", waitedtime_phase)
                            
                            threshold_start_at_timestep = step
                            threshold_start_at_timestepexit = step
                            waitedtime_phase_status=[0,0,0,0]
                            max_waitingline_phase = waitedtime_phase.index(max(waitedtime_phase))
                            if max_waitingline_phase != 1 and waitedtime_phase_status[max_waitingline_phase]==0 and max(waitedtime_phase) >= thresholdtimer:
                                print("threshold timer dijalankan pada phase ", max_waitingline_phase, "selama", green_duration, " detik")
                                dtt_phase.append(max_waitingline_phase)
                                dtt_count += 1
                                log_waitedtime.write(str(e) + ' ' + str(preemption_count) + ' ' + str(max(waitedtime_phase)) + ' \n')
                                
                                # log_waitedtime.write(str(max(waitedtime_phase)) + '\n')
                                
                                for idx, waited_phase in enumerate(waitedtime_phase) :
                                    waitedtime_phase[idx] = waited_phase+all_duration
                                waitedtime_phase[max_waitingline_phase] = 0
                                waitedtime_phase_status[max_waitingline_phase] = 1
                                threshold_start_at_timestep+=all_duration
                                print(waitedtime_phase)
                                print(waitedtime_phase_status)
                            max_waitingline_phase = waitedtime_phase.index(max(waitedtime_phase))
                            if max_waitingline_phase != 1 and waitedtime_phase_status[max_waitingline_phase]==0 and max(waitedtime_phase) >= thresholdtimer:
                                print("threshold timer akan dijalankan pada phase ", max_waitingline_phase, "selama", green_duration," detik")
                                dtt_phase.append(max_waitingline_phase)
                                dtt_count += 1
                                log_waitedtime.write(str(e) + ' ' + str(preemption_count) + ' ' + str(max(waitedtime_phase)) + ' \n')

                                # log_waitedtime.write(str(max(waitedtime_phase)) + '\n')
                                for idx, waited_phase in enumerate(waitedtime_phase) :
                                    waitedtime_phase[idx] = waited_phase+all_duration
                                waitedtime_phase[max_waitingline_phase] = 0
                                waitedtime_phase_status[max_waitingline_phase] = 1
                                threshold_start_at_timestep+=all_duration

                                print(waitedtime_phase)
                                print(waitedtime_phase_status)
                            max_waitingline_phase = waitedtime_phase.index(max(waitedtime_phase))
                            if max_waitingline_phase != 1 and waitedtime_phase_status[max_waitingline_phase]==0 and max(waitedtime_phase) >= thresholdtimer:
                                print("threshold timer dijalankan pada phase ", max_waitingline_phase, "selama",green_duration," detik")
                                dtt_phase.append(max_waitingline_phase)
                                dtt_count += 1
                                log_waitedtime.write(str(e) + ' ' + str(preemption_count) + ' ' + str(max(waitedtime_phase)) + ' \n')

                                # log_waitedtime.write(str(max(waitedtime_phase)) + '\n')
                                for idx, waited_phase in enumerate(waitedtime_phase) :
                                    waitedtime_phase[idx] = waited_phase+all_duration
                                waitedtime_phase[max_waitingline_phase] = 0
                                waitedtime_phase_status[max_waitingline_phase] = 1
                                threshold_start_at_timestep+=all_duration

                                print(waitedtime_phase)
                                
                                print(waitedtime_phase_status)
                            
                        else:
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
                            # get waiting line terpanjang ada di jalur mana
                            max_waitingline_phase=vehiclenumbers.index(max(vehiclenumbers))
                            # max_waitingline_phase=2
                            tls = [0,2,4,6]
                            print("exit phase ke max waiting line: phase: " + str(max_waitingline_phase))
                            log_waitingtime.write("episode ke "+ str(e) +" preemption ke "+ str(preemption_count) +" exit phase only ke max waiting line: phase: " + str(max_waitingline_phase) + "\n")
                            traci.trafficlight.setPhase("tls", tls[max_waitingline_phase])
                            exitstrategy_status=""
                            # print(exitstrategy_status)
                            # exit()
                            
            # jalankan exit strategy sesuai dengan pemilihan
            if preemption_status=="" and exitstrategy_status=="on":
                match chosen_exit_strategy:
                    case 0:
                        # 1. no exit strategy (jalankan lagi siklusnya)
                        # kembali ke siklus berjalan sebelumnya yang terhenti
                        if (exitstrategy_start_at_timestep <= step <= int(exitstrategy_start_at_timestep+current_trafficlightelapsed)):
                            traci.trafficlight.setPhase("tls", current_trafficlight)
                            traci.trafficlight.setPhaseDuration("tls", 1)
                            if current_trafficlight%2==0:
                                print("W", end = '')
                            else:
                                print("--", end = '')

                        else:
                            exitstrategy_status=""  
                    case 1:
                        # kembali ke siklus berjalan sebelumnya yang terhenti ditambah 30 untuk fixed phase 
                        if (exitstrategy_start_at_timestep <= step <= int(exitstrategy_start_at_timestep+green_duration)):
                            # max untuk AI
                            traci.trafficlight.setPhase("tls", phase[max_waitingline_phase][0])
                            print("#", end = '')
                            traci.trafficlight.setPhaseDuration("tls", 1)
                        if (exitstrategy_start_at_timestep+all_duration+1 <= step <= int(exitstrategy_start_at_timestep+all_duration+current_trafficlightelapsed)):
                            traci.trafficlight.setPhase("tls", current_trafficlight)
                            print("W", end = '')
                            traci.trafficlight.setPhaseDuration("tls", 1)
                        if step == int(exitstrategy_start_at_timestep+all_duration+current_trafficlightelapsed):
                            exitstrategy_status=""
                            new_state = sumoInt.getState()
                            new=state[0]-new_state[0]
                            reward = sum(new[0])[0]
                            
                            cum_reward += reward
                            if (float(reward)<0):
                                cum_negativereward += reward
                            agent.remember(state, action, reward, new_state, False)
                            print("\nStore reward ", reward, "into memory")
                            # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
                            if(len(agent.memory) > batch_size):
                                agent.replay(batch_size)
                    case 2:

                        if (enddwell_start_at_timestep <= step <= enddwell_start_at_timestep+coord_preempt_trafficlight_duration):
                            traci.trafficlight.setPhase("tls", coord_preempt_trafficlight)
                            print("W", end = '')
                    
                        else:
                            # traci.trafficlight.setPhase("tls", coord_preempt_trafficlight)
                            # traci.trafficlight.setPhaseDuration("tls", coord_preempt_trafficlight_duration)
                            exitstrategy_status=""
                            new_state = sumoInt.getState()
                            
                            new=state[0]-new_state[0]
                            reward = sum(new[0])[0]
                            
                            cum_reward += reward
                            if (float(reward)<0):
                                cum_negativereward += reward
                            
                            agent.remember(state, action, reward, new_state, False)
                            print("Store reward", reward, "into memory")
                            # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
                            if(len(agent.memory) > batch_size):
                                # print(len(agent.memory))
                                # print(batch_size)
                                agent.replay(batch_size)
                    case 3:

                        if (enddwell_start_at_timestep <= step <= enddwell_start_at_timestep+green_duration):
                            traci.trafficlight.setPhase("tls", phase[enddwell_phase[0]][0])
                            print("W", end = '')
                        if (enddwell_start_at_timestep+green_duration+1 <= step <= enddwell_start_at_timestep+green_duration+yellow_duration):
                            traci.trafficlight.setPhase("tls", phase[enddwell_phase[0]][1])
                            print("-", end = '')

                        if (enddwell_start_at_timestep+all_duration+1 <= step <= enddwell_start_at_timestep+all_duration+green_duration):
                            traci.trafficlight.setPhase("tls", phase[enddwell_phase[1]][0])
                            print("W", end = '')
                        if (enddwell_start_at_timestep+all_duration+green_duration+1 <= step <= enddwell_start_at_timestep+all_duration+green_duration+yellow_duration):
                            traci.trafficlight.setPhase("tls", phase[enddwell_phase[1]][1])
                            print("-", end = '')

                        if (enddwell_start_at_timestep+all_duration*2+1 <= step <= enddwell_start_at_timestep+all_duration*2+green_duration):
                            traci.trafficlight.setPhase("tls", phase[enddwell_phase[2]][0])
                            print("W", end = '')
                        if (enddwell_start_at_timestep+all_duration*2+green_duration+1 <= step <= enddwell_start_at_timestep+all_duration*2+green_duration+yellow_duration):
                            traci.trafficlight.setPhase("tls", phase[enddwell_phase[2]][1])
                            print("-", end = '')
                        

                        if (enddwell_start_at_timestep+red_duration*(all_duration+1) == step):
                            # get waiting line non EV di semua jalur ketika preemption selesai
                            vehiclenumbers=[
                                (int(traci.lane.getLastStepVehicleNumber("jljendsudirman_satuarah_2_0")+
                                traci.lane.getLastStepVehicleNumber("jljendsudirman_satuarah_2_1")+
                                traci.lane.getLastStepVehicleNumber("jljendsudirman_satuarah_2_2")+
                                traci.lane.getLastStepVehicleNumber("jljendsudirman_satuarah_1_0")+
                                traci.lane.getLastStepVehicleNumber("jljendsudirman_satuarah_1_1")+
                                traci.lane.getLastStepVehicleNumber("jljendsudirman_satuarah_1_2"))/3*2), 
                                
                                int(traci.lane.getLastStepVehicleNumber("jlsuroto_barat_1_0")+ 
                                traci.lane.getLastStepVehicleNumber("jlsuroto_barat_1_1")+ 
                                traci.lane.getLastStepVehicleNumber("jlsuroto_barat_2_1")+ 
                                traci.lane.getLastStepVehicleNumber("jlsuroto_barat_2_2")),

                                int(traci.lane.getLastStepVehicleNumber("jljendsudirman_utara_7_0")+ 
                                traci.lane.getLastStepVehicleNumber("jljendsudirman_utara_7_1"))
                                , 
                                int(traci.lane.getLastStepVehicleNumber("jlcikditiro_timur_1_0")+
                                traci.lane.getLastStepVehicleNumber("jlcikditiro_timur_1_1"))
                                ]
                            # get waiting line terpanjang ada di jalur mana
                            max_waitingline_phase=vehiclenumbers.index(max(vehiclenumbers))
                            print("Vehicle density : ", vehiclenumbers)
                            # print("MAXIMAL ADA DI PHASE : ", max_waitingline_phase)
                        
                        # exit phase pake yang fixed phase yaitu phase maksimal
                        if ((enddwell_start_at_timestep+3*all_duration) <= step <= enddwell_start_at_timestep+3*all_duration+green_duration-1):
                            # max untuk AI
                            traci.trafficlight.setPhase("tls", phase[max_waitingline_phase][0])
                            # fixed phasenya kita pilih arah selatan atau phase 6
                            # traci.trafficlight.setPhase("tls", 2)
                        if (enddwell_start_at_timestep+3*all_duration+green_duration-1 <= step <= enddwell_start_at_timestep+3*all_duration+green_duration-1+yellow_duration-1):
                            # max untuk AI
                            # traci.trafficlight.setPhase("tls", max_waitingline_phase+1)
                            # fixed phasenya kita pilih arah selatan atau phase 6
                            traci.trafficlight.setPhase("tls", 7)

                        if step == enddwell_start_at_timestep+3*all_duration+green_duration-1+yellow_duration:
                            es3_enddwell_status=""
                            print("\nServe non preempted phase : ", enddwell_phase)
                            print("Programmed exit phase ", max_waitingline_phase, "for ", all_duration," s")
                            print("Serve non preempted phase done in step : ", step)
                            
                            print("Run interrupted signal phase ", current_trafficlight, "for about ", current_trafficlightelapsed, "s")
                            
                            
                            # kembali ke siklus berjalan sebelumnya yang terhenti
                            traci.trafficlight.setPhase("tls", current_trafficlight)
                            traci.trafficlight.setPhaseDuration("tls", current_trafficlightelapsed)

                            exitstrategy_status=""
                            new_state = sumoInt.getState()
                            # print("state lama")
                            # print(state[0])
                            # print("state baru")
                            # print(new_state[0])
                            # print("dikurangi")
                            new=state[0]-new_state[0]
                            reward = sum(new[0])[0]

                            cum_reward += reward
                            if (float(reward)<0):
                                cum_negativereward += reward
                            agent.remember(state, action, reward, new_state, False)
                            print("Store reward ", reward, "into memory")
                            # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
                            if(len(agent.memory) > batch_size):
                                # print(len(agent.memory))
                                # print(batch_size)
                                agent.replay(batch_size)
                                
                    case 4:
                        # print("masuk ke case 4 yang kedua")
                        # print(waitedtime_phase)
                        # print(max(waitedtime_phase))
                        # print(thresholdtimer)
                        # print(max(waitedtime_phase) >= thresholdtimer, exitstrategy_status=="on")
                        for i in range(len(dtt_phase[:2])):    
                            # print(threshold_start_at_timestep*(i+1))
                            if (threshold_start_at_timestepexit+(all_duration*(i))+1 <= step <= threshold_start_at_timestepexit+(all_duration*(i))+green_duration):
                                traci.trafficlight.setPhase("tls", phase[dtt_phase[i]][0])
                                print("W", end = '')
                                # print("ijokan ke "+ str(i) + "detik ke " + str(step))
                            if (threshold_start_at_timestepexit+(all_duration*(i))+green_duration+1 <= step <= threshold_start_at_timestepexit+(all_duration*(i))+green_duration+yellow_duration):
                                print("-", end = '')
                                # print("kuningkan ke "+ str(i) + "detik ke " + str(step))
                                traci.trafficlight.setPhase("tls", phase[dtt_phase[i]][1])
                        # if (exitstrategy_start_at_timestep*(len(dtt_phase)+1) <= step <= int(exitstrategy_start_at_timestep*(len(dtt_phase)+1)+current_trafficlightelapsed-1)):
                        #     print("exit phase berjalan")
                        #     traci.trafficlight.setPhase("tls", current_trafficlight)
                        if (threshold_start_at_timestepexit+(all_duration*(2))+1 == step):
                            # exit phase ke max waiting line
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
                            # get waiting line terpanjang ada di jalur mana
                            max_waitingline_phase=vehiclenumbers.index(max(vehiclenumbers))
                            max_waitingline_phase=0
                            tls = [0,2,4,6]
                            log_waitingtime.write("episode ke "+ str(e) +" preemption ke "+ str(preemption_count) +" exit phase ke max waiting line: phase: " + str(max_waitingline_phase) + "\n")
                            print("\nProgrammed exit phase ", max_waitingline_phase, "for : ", all_duration," s")
                            traci.trafficlight.setPhase("tls", tls[max_waitingline_phase])
                            exitstrategy_status="" 
                            new_state = sumoInt.getState()
                            new=state[0]-new_state[0]
                            reward = sum(new[0])[0]

                            cum_reward += reward
                            if (float(reward)<0):
                                cum_negativereward += reward
                            agent.remember(state, action, reward, new_state, False)
                            print("Store reward ", reward, "into memory")
                            # Randomly Draw 32 samples and train the neural network by RMS Prop algorithm
                            if(len(agent.memory) > batch_size):
                                # print(len(agent.memory))
                                # print(batch_size)
                                agent.replay(batch_size)
                            dtt_phase.clear()

                

                
               
            
            if preemption_timer!=0:
                # print(preemption_timer)
                none=""
            else: 
                preemption_status=""
                
            # satu intersection selesai
            

            # print(traci.trafficlight.getPhaseDuration('tls'))
            get_timeremaining_phase = traci.trafficlight.getNextSwitch('tls')-traci.simulation.getTime()
            
            step += 1
        # normal vehicle count, ev count, cumulative preemption duration, average waiting time,
        nev_count = math.ceil(1800/period_nonev)
        avg_waitingtime = round(waiting_time/(nev_count+ev_count), 2)
        avg_preemptionduration=0
        if (cum_preemption!=0):
            avg_preemptionduration = round(cum_preemption/(ev_count), 0)
        log_episode.write(str(nev_count) + " " + str(preemption_count) + " " + str(avg_preemptionduration) + " " + str(count_es1)+ " " + str(count_es2)+ " " + str(count_es3)+ " " + str(count_es4)+ " " + str(count_es5) + " " + str(cum_reward) + " " + str(cum_negativereward) + " " + str(avg_waitingtime) + " " + '\n')
        # log_waitingtime.write(str(waiting_time) + ' ' + str(dtt_count) + ' \n')
     
        print("\nEnd this episode in step ", step, "\n \n")
        print('episode - ' + str(e))
        traci.close(wait=False)

sys.stdout.flush()