// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Adapter
#define Ow_Adapter

// PLEXIL Interface adapter for OceanWATERS.

// PLEXIL API
#include "Command.hh"
#include "InterfaceAdapter.hh"
#include "Value.hh"
#include "SimpleMap.hh"
#include "Update.hh"

#include <set>

using namespace PLEXIL;

class OwAdapter : public InterfaceAdapter
{
public:
  typedef SimpleMap<std::string, Value> PairValueMap;

  // No default constructor, only this specialized one.
  OwAdapter (AdapterExecInterface&, const pugi::xml_node&);
  ~OwAdapter ();
  OwAdapter (const OwAdapter&) = delete;
  OwAdapter& operator= (const OwAdapter&) = delete;

  virtual bool initialize();
  virtual bool start();
  virtual bool stop();
  virtual bool reset();
  virtual bool shutdown();
  virtual void invokeAbort(Command *cmd);
  virtual void lookupNow (State const& state, StateCacheEntry &entry);
  virtual void subscribe(const State& state);
  virtual void unsubscribe(const State& state);
  //virtual void sendPlannerUpdate(Update *update);

  static void myPlannerUpdate (PLEXIL::Update* update, PLEXIL::AdapterExecInterface* exec);
  void propagateValueChange (const State&, const std::vector<Value>&) const;

private:
  bool isStateSubscribed(const State& state) const;
  std::set<State> m_subscribedStates;
};

extern "C" {
  void initOwAdapter();
}

#endif
