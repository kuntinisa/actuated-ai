import os 
import sys 
import optparse

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

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
    options, args = opt_parser.parse_args()
    return options

#define the shape of the environment (i.e., its states)
#waiting time di tiap phase bisa dihitung maksimal hingga 10 menit atau 600 detik.
#sehingga 4 phase ada 2400 states
states_waitingtime_4phases = 2400
#waiting line tiap phase bisa dihitung maksimal hingga 10 meter.
#sehingga 4 phase ada 40 states
states_waitingline_4phases = 40
#tbc timer maksimal satu siklus dalam persimpangan. Misal tiap phase 2 menit berarti siklus selesai dalam waktu 8 menit atau 480 detik
states_tbc_intersection = 480
#ada 3 states berarti akan dibuat array 3 dimensi sehingga ada 2400x40x480 = 46.080.000 states


#Create a 3D numpy array to hold the current Q-values for each state and action pair: Q(s, a) 
#The "action" dimension consists of 4 layers that will allow us to keep track of the Q-values for each possible action in
#each state (see next cell for a description of possible actions). 
#The value of each (state, action) pair is initialized to 0.
#actionnya ada 4x4 + 4x300 = 136

q_values = np.zeros((states_waitingtime_4phases, states_waitingline_4phases, states_tbc_intersection, 136))

#exit strategy
#exit phase
#threshold timer rentang 0 - 600 detik
actions = [['no exit strategy', 'fixed phase strategy', 'coord+preempt', 'dwell phase', 'threshold timer'],['dynamic', 'fixed'],[*range(0, 600, 1)]]
            
#Calculate reward and punishment
# Reward ditentukan oleh beberapa kondisi diantaranya waiting time dari vehicle yang berada di persimpangan tersebut
# By default waiting time per phase adalah waktu siklus dikurangi dengan durasi hijau dari phase tersebu
# Misal tiap phase 2 menit maka satu siklus ada 8 menit
# Minimal waiting time kendaraan adalah 6 menit
# Ini hanya perhitungan awal
# Kita harus mencari tau waiting time rata2 sebelum training
# Jika waiting time kendaraan setelah dipasang AI lebih kecil dari standar maka reward berupa plus selisihnya
# Jika waiting time kendaraan setelah dipasang AI lebih besar dari standar maka reward berupa minus selisihnya
standard_waitingtime = [360,360,360,360]

# Initiate reward for each state = 0;
rewards = np.full((states_waitingtime_4phases, states_waitingline_4phases, states_tbc_intersection), 0)

# DEFINE HELPER FUNCTION IN EACH INTERSECTION

# 1. state 
# get waiting time di tiap phase
states_waitingtime_4phases = [0,0,0,0]
# get waiting line di tiap phase
states_waitingline_4phases = [0,0,0,0]

# 2. action
# apa yang bisa dilakukan di tiap intersection?
# contoh
exit_strategy = 'no exit strategy'
exit_phase = 'dynamic' #nanti AI yang cari pake helper phase yang mana
threshold_timer = 0
actions = [exit_strategy, exit_phase, threshold_timer]

# 3. reward
# reward ini diformulasi dengan rumus untuk mendapatkan angka reward yang tinggi
# karna reward dipengaruhi oleh waiting time non ev
# angkanya didapat dari sumo waiting time kendaraan berapa
average_wating_time_impact = [360,360,360,360];

# HELPER FUNCTION AI
# 1. Kalkulasi q_value ada di reward lalu masukkan ke 
 
# 2. Save nilai reward dalam matriks q_value atau rewards
rewards[0,0,0] = 0;

# HELPER FUNCTION PREEMPTION

# exit strategy : threshold timer
thresholdtimer = 0;

# get siklus sekarang
simpang_name = 'simpang_gramedia'
def get_current_tlscycle(simpang_name):
    # return state, phase, timer of state, tbs
    return ['green', utara, 12, 12]

# get preemption mulai di phase mana
def get_current_preemption(simpang_name):
    # return state, phase, timer of preemption
    return ['on-preemption', utara, 5]

# get ambulance random muncul di rute mana dan kapan
def get_starting_location():
  #get a random route
  all_route = ['a', 'b', 'c', 'd', 'e']
  current_route_preemption = np.array(list(all_route))
  return current_route_preemption

#define an epsilon greedy algorithm that will choose which action to take next (i.e., where to move next)
def get_next_action(epsilon):
  #if a randomly chosen value between 0 and 1 is less than epsilon, 
  #then choose the most promising value from the Q-table for this state.
  if np.random.random() < epsilon:
    return np.argmax(q_values[current_row_index, current_column_index])
    q_values = np.zeros((states_waitingtime_4phases, states_waitingline_4phases, states_tbc_intersection))
  else: #choose a random action
    return np.array(list(actions))
  

  #helper untuk action 
  def action_thresholdtimer():
      return true


# TRAIN THE MODEL IN THE LOOP
# contains TraCI control loop
def run ():
    step = 0

        # green preemption gramedia 0,2,4,6
        # green preemption tugu: tls_tugu_timur 1, tls_tugu_utara 1, tls_tugu_barat 1, tls_tugu_selatan 0
        
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        # preemption process in simpang gramedia - gramedia timur
        if traci.inductionloop.getLastStepVehicleNumber("evdetector0") > 0:
            print("Ambulance detected in 0")
            if traci.trafficlight.getPhase("tls_gramedia") != 0:
                print("ubah ke hijau semua")
                traci.trafficlight.setPhase("tls_gramedia", 0)
            # lampu merah diganti fasenya
        if traci.inductionloop.getLastStepVehicleNumber("evdetector1") > 0:
            print("Ambulance detected in 1")
            if traci.trafficlight.getPhase("tls_gramedia") != 0:
                print("ubah ke hijau semua")
                traci.trafficlight.setPhase("tls_gramedia", 0)
        if traci.inductionloop.getLastStepVehicleNumber("evdetector2") > 0:
            print("Ambulance detected in 2")
            if traci.trafficlight.getPhase("tls_gramedia") != 0:
                print("ubah ke hijau semua")
                traci.trafficlight.setPhase("tls_gramedia", 0)

        # AI works
        #define training parameters
        epsilon = 0.9 #the percentage of time when we should take the best action (instead of a random action)
        discount_factor = 0.9 #discount factor for future rewards
        learning_rate = 0.9 #the rate at which the AI agent should learn

        #choose which action to take (i.e., where to move next)
        action_index = get_next_action(epsilon)

        #perform the chosen action, and transition to the next state (i.e., move to the next location)
        # suruh SUMO lakukan itu
        
        #receive the reward for moving to the new state, and calculate the temporal difference
        reward = rewards[states_waitingtime_4phases, states_waitingline_4phases, states_tbc_intersection]

        old_q_value = q_values[states_waitingtime_4phases, states_waitingline_4phases, states_tbc_intersection, action_index]
        temporal_difference = reward + (discount_factor * np.max(q_values[states_waitingtime_4phases, states_waitingline_4phases, states_tbc_intersection])) - old_q_value

        #update the Q-value for the previous state and action pair
        new_q_value = old_q_value + (learning_rate * temporal_difference)
        q_values[states_waitingtime_4phases, states_waitingline_4phases, states_tbc_intersection, action_index] = new_q_value


        step + 1
    traci.close()
    sys.stdout.flush()
    
    # main entry point
if __name__ == "__main__":
    options = get_options()
# check binary
if options.nogui:
    sumoBinary = checkBinary('sumo')
else:
    sumoBinary = checkBinary('sumo-gui')
# traci starts sumo as a subprocess and then this script connects and runs
traci.start([sumoBinary, "-c", "osm.sumocfg"])
run()