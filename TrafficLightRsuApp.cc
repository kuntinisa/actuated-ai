#include "veins/modules/application/leee80211p/BaseWaveApplLayer.h" 
#include "veins/modules/mobility/traci/TraCIScenarioManager.h" 
#include "veins/modules/mobility/traci/TraCICommandInterface.h"


namespace Veins{
class TrafficLightsuApp : public BaseWaveApplLayer {
protected:
    virtual void initialize(int stage);
    virtual void onWSM(WaveShortMessage* wsm);
    virtual void onWSA(WaveServiceAdvertisment* wsa);
    virtual void onBSM(BasicSafetyMessage * bsm);
    virtual void handleSelfMsg(cMessage* msg);
    TraCIScenarioManager* manager; std::string trafficLightId;
    Message* initMsg;
    Message* phaseMsg;
};
}