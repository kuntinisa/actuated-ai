import os 
import sys 
import optparse
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import math as math

# Evaluasi
# 1. 5996
# 2. 5996
# 3. 6155
# 4. 6341
# 5. 6352

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



# TRAIN THE MODEL IN THE LOOP
# contains TraCI control loop
def run ():
    step = 0
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
    all_duration = 30
    green_duration = 24
    yellow_duration = 6 



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


    # useless variabel
    none=""


    # buat fungsi untuk cek semua detector, cek apakah ada EV yang melintas atau tidak
    for persimpangan in persimpangans:
        if persimpangan=="gramedia":
            for idx, phase in enumerate(phases) :
                if persimpangan=="gramedia" and phase=="timur":
                    for x in range(3) :
                        print("in_"+persimpangan+phase+str(x+1))
                else:
                    for x in range(2) :
                        print("in_"+persimpangan+phase+str(x+1))
                    for x in range(2) :
                        print("out_"+persimpangan+phase+str(x+1))   

    max_waitingline_phase=none;
    



    # print(persimpangan)

    evdetectortimur=["in_gramediatimur1", "in_gramediatimur2", "in_gramediatimur3"]
    evdetectortugu=["in_tugu1", "in_tugu2"]
    evdetectorpingit=["inpingit_1", "in_pingit2"]
    out_evdetectortimur=["out_gramediabarat1", "out_gramediabarat2"]
    out_evdetectortugu=["in_tugu1", "in_tugu2"]
    out_evdetectorpingit=["inpingit_1", "in_pingit2"]
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        # disini perulangan di tiap intersection

        # deteksi EV masuk dengan inductionloop detector
        for evdetector in evdetectortimur:
            # jika EV terdeteksi oleh induction loop
            if traci.inductionloop.getLastStepVehicleNumber(evdetector) > 0:
                # jalankan preemption saat tidak ada preemption lain
                if preemption_status!="on":
                    # get phase tls saat EV terdeteksi (0,1,2,3,4,5,6,7)
                    current_trafficlight = traci.trafficlight.getPhase("tls_gramedia")
                    # get duration tls saat EV terdeteksi (second)
                    current_trafficlightduration = traci.trafficlight.getPhaseDuration("tls_gramedia")
                    # get duration tersisa tls saat EV terdeteksi
                    current_trafficlightelapsed = traci.trafficlight.getNextSwitch("tls_gramedia")-traci.simulation.getTime()
                    # menentukan jalur / phase jalanan tls saat EV terdeteksi (0,1,2,3)
                    phase=[0,1,6,7,4,5,2,3]
                    now_phase = math.ceil((phase.index(current_trafficlight)+1)/2)-1
                    phase_cycle = [0,1,2,3]
                    tls_cycles_part = [0,1,2,3,4,5,6,7]
                    tls_cycle_times = [24,6,24,6,24,6,24,6]
                    # get duration yang telah dijalankan tls saat EV terdeteksi
                    # hitung tls sekarang udah jalan berapa detik
                    if current_trafficlight%2 == 0:
                        tls_done_duration = 24-current_trafficlightelapsed
                    else:
                        tls_done_duration = 30-current_trafficlightelapsed

                    print("\n \n ====================================================")
                    print("PREEMPTION ON dideteksi oleh ", evdetector)
                    print("menghentikan tls ", current_trafficlight, " di phase ",now_phase," masih tersisa waktu ", current_trafficlightelapsed, " detik, sudah berjalan ", tls_done_duration, " detik")
                    
                    
                    # kalkulasi waktu tunggu jalur lain non preempt
                    phase=[[0,1], [6,7], [4,5], [2,3]]
                    waitedtime_phase=[0,0,0,0]
                    waitedtime_phase[now_phase] = 0
                    # next array
                    now_phase = (now_phase + 1) % len(phase_cycle)
                    waitedtime_phase[now_phase] = 90-30*1+tls_done_duration
                    # next array
                    now_phase = (now_phase + 1) % len(phase_cycle)
                    waitedtime_phase[now_phase] = 90-30*2+tls_done_duration
                    # next array
                    now_phase = (now_phase + 1) % len(phase_cycle)
                    waitedtime_phase[now_phase] = 90-30*3+tls_done_duration
                    # karna preemption berjalan di phase 0 maka 0 kan phase ini
                    waitedtime_phase[0] = 0
                    print("non preempt phase waited time: ", waitedtime_phase)
                    
                    # set tls ke yellow untuk phase saat EV terdeteksi 
                    # if 0,2,4,6 maka jadikan kuning dulu 0->1, 2->3 selama 6 detik
                    match traci.trafficlight.getPhase("tls_gramedia"):
                        # case 0 adalah phase jalur yang dilewati EV
                        # case 0:
                            # traci.trafficlight.setPhaseDuration("tls_gramedia", 1, 6)
                            # traci.trafficlight.setPhase("tls_gramedia", 0)
                        case 2:
                            traci.trafficlight.setPhase("tls_gramedia", 3)
                            traci.trafficlight.setPhaseDuration("tls_gramedia", 6)
                        case 4:
                            traci.trafficlight.setPhase("tls_gramedia", 5)
                            traci.trafficlight.setPhaseDuration("tls_gramedia", 6)
                        case 6:
                            traci.trafficlight.setPhase("tls_gramedia", 7)
                            traci.trafficlight.setPhaseDuration("tls_gramedia", 6)
                    
                    # preemption green tls dimulai pertama kali
                    traci.trafficlight.setPhase("tls_gramedia", 0)
                    traci.trafficlight.setPhaseDuration("tls_gramedia", 2)

                    # kebutuhan ES coord_preempt menyelaraskan tbc dan loc
                    coord_preempt_trafficlight_duration = current_trafficlightelapsed
                    coord_preempt_trafficlight = current_trafficlight

                    # jika ada EV yang terdeteksi maka ubah status preemption ke on 
                    if traci.inductionloop.getLastStepVehicleIDs(evdetector) != "()":
                        vehicle_on_preemption.append(traci.inductionloop.getLastStepVehicleIDs(evdetector))
                        preemption_status="on"
                        print("EV yang sedang melintas : ", vehicle_on_preemption, "\n")
        # selesai mendeteksi di semua detector masuk

        # preemption sedang berlangsung            
        if preemption_status=="on":
            # green tls jalur preemption selama status preemption on
            traci.trafficlight.setPhase("tls_gramedia", 0)
            traci.trafficlight.setPhaseDuration("tls_gramedia", 2)
            # hitung lamanya preemption berlangsung
            preemption_timer += 1

            # ES coord_preempt kurangi duration untuk menyelaraskan tbc dan loc
            coord_preempt_trafficlight_duration -= 1 
            tls_cycles_part = [0,1,2,3,4,5,6,7]
            tls_cycle_times = [24,6,24,6,24,6,24,6]
            if coord_preempt_trafficlight_duration==0:
                # next array
                next = (coord_preempt_trafficlight + 1) % len(tls_cycles_part)
                # print("phase selanjutnya : ", next)
                coord_preempt_trafficlight = next
                coord_preempt_trafficlight_duration=tls_cycle_times[next]
            
        # deteksi EV keluar dengan induction loop
        for evdetector in out_evdetectortimur:
            # jika ada EV yang terdeteksi keluar
            if traci.inductionloop.getLastStepVehicleNumber(evdetector):
                # jika EV tersebut benar sama dengan yang dideteksi oleh induction loop sebelumnya
                if traci.inductionloop.getLastStepVehicleIDs(evdetector) in vehicle_on_preemption:
                    print("EV keluar dideteksi oleh evdetector ", evdetector)
                    # remove EV dari daftar EV yang sedang preemption
                    vehicle_on_preemption.remove(traci.inductionloop.getLastStepVehicleIDs(evdetector))
                    # Jika EV dalam daftar sudah habis
                    if not vehicle_on_preemption:
                        # catat lama preemption
                        preemption_timer_stop_at = preemption_timer
                        # ubah status preemption
                        preemption_status="done"
                        # yellow tls selama 6 detik sebelum ke ES
                        traci.trafficlight.setPhase("tls_gramedia", 1)


        # jika semua ev telah melintas, matikan timer preemption 
        if not vehicle_on_preemption:
            preemption_timer = 0;
        
        # jika preemption selesai, pilih exit strategy
        if preemption_status=="done" :
            # hitung ini preemption ke berapa
            preemption_count+=1
            print("PREEMPTION[", preemption_count, "] ", preemption_status, " dalam waktu ", preemption_timer_stop_at, " detik \n")

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
                traci.lane.getLastStepVehicleNumber("jlsuroto_barat_2_2")+ 
                traci.lane.getLastStepVehicleNumber("jlsuroto_barat_3_0")+ 
                traci.lane.getLastStepVehicleNumber("jlsuroto_barat_3_1")),

                int(traci.lane.getLastStepVehicleNumber("jljendsudirman_utara_7_0")+ 
                traci.lane.getLastStepVehicleNumber("jljendsudirman_utara_7_1")), 

                int(traci.lane.getLastStepVehicleNumber("jlcikditiro_timur_1_0")+
                traci.lane.getLastStepVehicleNumber("jlcikditiro_timur_1_1"))
                ]
            # get waiting line terpanjang ada di jalur mana
            max_waitingline_phase=vehiclenumbers.index(max(vehiclenumbers))

            print("EXIT STRATEGY DECISION MAKING using AI")
            exit_strategy=['1. no_exit_phase', '2. fixed_phase_max_waitingline', '3. coord_preempt', '4. end dwell', '5. dynamic threshold timer']

            print("AI seharusnya berjalan disini untuk menentukan ES mana yang akan dijalankan")
            chosen_exit_strategy = 0
            print("exit strategy yang pilih : ", exit_strategy[chosen_exit_strategy])
            
            # jalankan exit strategy yang telah dipilih
            exitstrategy_status = "on"
            exitstrategy_start_at_timestep = step


            match chosen_exit_strategy:
                case 0:
                    print("kembali ke cycle awal phase ", current_trafficlight, "selama ", current_trafficlightelapsed, "detik")
                case 1:
                    # 2. fixed phase exit strategy 
                    
                    print("tingkat kemacetan di semua phase : ", vehiclenumbers)
                    print("2. fixedphase exit strategy ke phase waiting line terpanjang : phase ", max_waitingline_phase, "selama : 30 detik")
                    print("lalu kembali ke cycle awal phase ", current_trafficlight, "selama ", current_trafficlightelapsed, "detik")

                case 2:
                    # 3. coord_preempt menghitung lama preemption akan berhenti ketika phase apa
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
                    print("3. coord_preempt exit strategy ke phase : ", coord_preempt_trafficlight, "selama", coord_preempt_trafficlight_duration, " detik")
                    traci.trafficlight.setPhase("tls_gramedia", coord_preempt_trafficlight)
                    traci.trafficlight.setPhaseDuration("tls_gramedia", coord_preempt_trafficlight_duration)
                    print("ga perlu kembali ke cycle awal ", current_trafficlight, "selama : ", current_trafficlightelapsed, "detik, tapi lanjutkan karna tbc dan loc udah sesuai")
                    # kembali ke tls yang terhenti karna preemption
                    # traci.trafficlight.setPhase("tls_gramedia", current_trafficlight)
                    # traci.trafficlight.setPhaseDuration("tls_gramedia", current_trafficlightelapsed)

                case 3:
                    exitstrategy_status = "on"
                    # 3. end dwell 
                    print("tingkat kemacetan di semua phase : ", vehiclenumbers)
                    print("jalankan lampu hijau sesuai urutan kemacetan skip phase 0")
                    
                    # get urutan non preempt phase dari tertinggi ke terendah
                    maxList = vehiclenumbers

                    temp = ([i[0] for i in sorted(enumerate(maxList), key=lambda k: k[1], reverse=True)])
                    # hilangkan phase 0
                    temp.remove(0)
                    enddwell_phase = temp
                    print(temp)
                    print("enddwell dimulai di step: ", step)
                    enddwell_start_at_timestep=step

                case 4:
                    # cek non preempt phase menunggu berapa lama
                    # tetapkan threshold misal 90+10 detik
                    # ini yang dibuat AI milih sendiri threshold timer berapa
                    # range 90-180 
                    thresholdtimer = 120


                    for idx, waited_phase in enumerate(waitedtime_phase) :
                        waitedtime_phase[idx] = waited_phase+preemption_timer_stop_at
                    print(waitedtime_phase)

                    
                    # while max(waitedtime_phase) >= thresholdtimer:
                    if max(waitedtime_phase) >= thresholdtimer:
                        exitstrategy_status = "on"
                        print("threshold memenuhi ketika step : ", step)
                        threshold_start_at_timestep = step
                        # exit()
                    else:
                        print("threshold tidak terpenuhi, kembali ke siklus normal")
                        print("kembali ke cycle awal phase ", current_trafficlight, "selama ", current_trafficlightelapsed, "detik")

                        # kembali ke siklus berjalan sebelumnya yang terhenti
                        traci.trafficlight.setPhase("tls_gramedia", current_trafficlight)
                        traci.trafficlight.setPhaseDuration("tls_gramedia", current_trafficlightelapsed)

        
        # jalankan exit strategy sesuai dengan pemilihan
        if preemption_status=="" and exitstrategy_status=="on":
            match chosen_exit_strategy:
                case 0:
                    # 1. no exit strategy (jalankan lagi siklusnya)
                    # kembali ke siklus berjalan sebelumnya yang terhenti
                    if (exitstrategy_start_at_timestep <= step <= int(exitstrategy_start_at_timestep+current_trafficlightelapsed-1)):
                        traci.trafficlight.setPhase("tls_gramedia", current_trafficlight)
                    else:
                        exitstrategy_status=""
                case 1:
                    phase=[[0,1], [6,7], [4,5], [2,3]]
                    # kembali ke siklus berjalan sebelumnya yang terhenti ditambah 30 untuk fixed phase 
                    if (exitstrategy_start_at_timestep <= step <= int(exitstrategy_start_at_timestep+24-1)):
                        traci.trafficlight.setPhase("tls_gramedia", phase[max_waitingline_phase][0])
                        traci.trafficlight.setPhaseDuration("tls_gramedia", 1)
                    if (exitstrategy_start_at_timestep+30 <= step <= int(exitstrategy_start_at_timestep+30+current_trafficlightelapsed-1)):
                        traci.trafficlight.setPhase("tls_gramedia", current_trafficlight)
                        traci.trafficlight.setPhaseDuration("tls_gramedia", 1)
                    if step == int(exitstrategy_start_at_timestep+30+current_trafficlightelapsed):
                        exitstrategy_status=""
                # case 2:
                case 3:
                    # phase 1
                    # 72+23*0+5*0, 72+23*1+5*0
                    # 72+23*1+5*0, 72+23*1+5*1

                    # phase 2
                    # 72+23*1+5*1, 72+23*2+5*1
                    # 72+23*2+5*1, 72+23*2+5*2
                    phase=[[0,1], [6,7], [4,5], [2,3]]
                    
                    # enddwell_phase[0]
                    if (enddwell_start_at_timestep <= step <= enddwell_start_at_timestep+green_duration-1):
                        traci.trafficlight.setPhase("tls_gramedia", phase[enddwell_phase[0]][0])
                    if (enddwell_start_at_timestep+green_duration-1 <= step <= enddwell_start_at_timestep+green_duration-1+yellow_duration-1):
                        traci.trafficlight.setPhase("tls_gramedia", phase[enddwell_phase[0]][1])
                    if (enddwell_start_at_timestep+30 <= step <= enddwell_start_at_timestep+all_duration+green_duration-1):
                        traci.trafficlight.setPhase("tls_gramedia", phase[enddwell_phase[1]][0])
                    if (enddwell_start_at_timestep+all_duration+green_duration-1 <= step <= enddwell_start_at_timestep+all_duration+green_duration-1+yellow_duration-1):
                        traci.trafficlight.setPhase("tls_gramedia", phase[enddwell_phase[1]][1])
                    if (enddwell_start_at_timestep+2*all_duration <= step <= enddwell_start_at_timestep+2*all_duration+green_duration-1):
                        traci.trafficlight.setPhase("tls_gramedia", phase[enddwell_phase[2]][0])
                    if (enddwell_start_at_timestep+2*all_duration+green_duration-1 <= step <= enddwell_start_at_timestep+2*all_duration+green_duration-1+yellow_duration-1):
                        traci.trafficlight.setPhase("tls_gramedia", phase[enddwell_phase[2]][1])
                    # exit phase pake yang fixed phase yaitu phase 0
                    if (enddwell_start_at_timestep+3*all_duration <= step <= enddwell_start_at_timestep+3*all_duration+green_duration-1):
                        traci.trafficlight.setPhase("tls_gramedia", phase[enddwell_phase[2]][0])
                    if (enddwell_start_at_timestep+3*all_duration+green_duration-1 <= step <= enddwell_start_at_timestep+3*all_duration+green_duration-1+yellow_duration-1):
                        traci.trafficlight.setPhase("tls_gramedia", phase[enddwell_phase[2]][1])

                    if step == enddwell_start_at_timestep+3*all_duration+green_duration-1+yellow_duration:
                        es3_enddwell_status=""
                        print("non preempt phase dijalankan di phase : ", enddwell_phase)
                        print("exit to fixed phase : 0")
                        print("enddwell phase berhenti di step : ", step)
                        print("kembali ke cycle awal phase ", current_trafficlight, "selama ", current_trafficlightelapsed, "detik")

                        # kembali ke siklus berjalan sebelumnya yang terhenti
                        traci.trafficlight.setPhase("tls_gramedia", current_trafficlight)
                        traci.trafficlight.setPhaseDuration("tls_gramedia", current_trafficlightelapsed)
                case 4:
                    waitedtime_phase_status=[0,0,0,0]

                    phase=[[0,1], [6,7], [4,5], [2,3]]

                    max_waitingline_phase = waitedtime_phase.index(max(waitedtime_phase))
                    if max_waitingline_phase != 0 and waitedtime_phase_status[max_waitingline_phase]==0 and max(waitedtime_phase) >= thresholdtimer and thresholdtimer_status=="on":
                        print("threshold timer dijalankan pada phase ", max_waitingline_phase, "selama 30 detik")

                        if (threshold_start_at_timestep <= step <= threshold_start_at_timestep+green_duration-1):
                            traci.trafficlight.setPhase("tls_gramedia", phase[max_waitingline_phase][0])
                        if (threshold_start_at_timestep+green_duration-1 <= step <= threshold_start_at_timestep+green_duration-1+yellow_duration-1):
                            traci.trafficlight.setPhase("tls_gramedia", phase[max_waitingline_phase][1])
                    
                        for idx, waited_phase in enumerate(waitedtime_phase) :
                            waitedtime_phase[idx] = waited_phase+30
                        waitedtime_phase[max_waitingline_phase] = 0
                        waitedtime_phase_status[max_waitingline_phase] = 1
                        threshold_start_at_timestep+=30

                        print(waitedtime_phase)
                        print(waitedtime_phase_status)
                    max_waitingline_phase = waitedtime_phase.index(max(waitedtime_phase))
                    if max_waitingline_phase != 0 and waitedtime_phase_status[max_waitingline_phase]==0 and max(waitedtime_phase) >= thresholdtimer and thresholdtimer_status=="on":
                        print("threshold timer dijalankan pada phase ", max_waitingline_phase, "selama 30 detik")

                        if (threshold_start_at_timestep <= step <= threshold_start_at_timestep+green_duration-1):
                            traci.trafficlight.setPhase("tls_gramedia", phase[max_waitingline_phase][0])
                        if (threshold_start_at_timestep+green_duration-1 <= step <= threshold_start_at_timestep+green_duration-1+yellow_duration-1):
                            traci.trafficlight.setPhase("tls_gramedia", phase[max_waitingline_phase][1])
                    
                        for idx, waited_phase in enumerate(waitedtime_phase) :
                            waitedtime_phase[idx] = waited_phase+30
                        waitedtime_phase[max_waitingline_phase] = 0
                        waitedtime_phase_status[max_waitingline_phase] = 1
                        threshold_start_at_timestep+=30

                        print(waitedtime_phase)
                        print(waitedtime_phase_status)
                    max_waitingline_phase = waitedtime_phase.index(max(waitedtime_phase))
                    if max_waitingline_phase != 0 and waitedtime_phase_status[max_waitingline_phase]==0 and max(waitedtime_phase) >= thresholdtimer and thresholdtimer_status=="on":
                        print("threshold timer dijalankan pada phase ", max_waitingline_phase, "selama 30 detik")

                        if (threshold_start_at_timestep <= step <= threshold_start_at_timestep+green_duration-1):
                            traci.trafficlight.setPhase("tls_gramedia", phase[max_waitingline_phase][0])
                        if (threshold_start_at_timestep+green_duration-1 <= step <= threshold_start_at_timestep+green_duration-1+yellow_duration-1):
                            traci.trafficlight.setPhase("tls_gramedia", phase[max_waitingline_phase][1])
                    
                        for idx, waited_phase in enumerate(waitedtime_phase) :
                            waitedtime_phase[idx] = waited_phase+30
                        waitedtime_phase[max_waitingline_phase] = 0
                        waitedtime_phase_status[max_waitingline_phase] = 1
                        threshold_start_at_timestep+=30

                        print(waitedtime_phase)
                        print(waitedtime_phase_status)
                    
                    print("threshold selesai________________________")
                    print(waitedtime_phase_status)
                    thresholdtimer_status=""

                    print("kembali ke cycle awal phase ", current_trafficlight, "selama ", current_trafficlightelapsed, "detik")

                    # kembali ke siklus berjalan sebelumnya yang terhenti
                    traci.trafficlight.setPhase("tls_gramedia", current_trafficlight)
                    traci.trafficlight.setPhaseDuration("tls_gramedia", current_trafficlightelapsed)

        if preemption_timer!=0:
            # print(preemption_timer)
            none=""
        else: 
            preemption_status=""
        # satu intersection selesai

        # print(traci.trafficlight.getPhaseDuration('tls_gramedia'))
        get_timeremaining_phase = traci.trafficlight.getNextSwitch('tls_gramedia')-traci.simulation.getTime()
        
       
        step += 1
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