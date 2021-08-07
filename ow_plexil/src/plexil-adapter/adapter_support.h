// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef adapter_support
#define adapter_support

// PLEXIL interface adapter support utilities.

// NOTE: In principle, this code could be folded into the CommonAdapter class,
// but the effort and result could be complicated in several ways.

#include "CommonAdapter.h"

// PLEXIL
#include <Value.hh>
#include <Command.hh>
#include <AdapterExecInterface.hh>
using namespace PLEXIL;

// C++
#include <map>
#include <string>


///////////////////////////// Conveniences //////////////////////////////////

// A prettier name for the "unknown" value.
const Value Unknown;

// A pointer to the interface adapter, so that it can be accessed from static
// functions.  WARNING: this allows only one adapter instance to be usable at a
// time; multiple testbeds cannot operate concurrently.
extern CommonAdapter* g_adapter;


//////////////////////////// Command Handling //////////////////////////////

// Unique ID for every instance of a command from a Plexil plan.
extern int CommandId;

// Record that combines a Plexil command instance with its executive interface
// and a flag indicating whether the command has been acknowledged.
using CommandRecord =
  std::tuple<Command*, AdapterExecInterface*, bool /* ack_sent */>;

enum CommandRecordFields {CR_COMMAND, CR_ADAPTER, CR_ACK_SENT};

std::unique_ptr<CommandRecord>&
new_command_record(Command*, AdapterExecInterface*);

// Registry of all commands currently in execution by testbed.
extern std::map<int, std::unique_ptr<CommandRecord>> CommandRegistry;

// Acknowledge a command issued by a Plexil plan, in a way that guarantees only
// one acknowledgment (acks are not idempotent).
void send_ack_once(CommandRecord&, bool skip=false);

// Function to call when a command finishes execution in testbed.
void command_status_callback (int id, bool success);

// "Receivers" for the pub/sub mechanism in subscriber.h
void receiveBool (const std::string& state_name, bool val);
void receiveDouble (const std::string& state_name, double val);
void receiveString (const std::string& state_name, const std::string& val);
void receiveBoolString (const std::string& state_name,
                        bool val,
                        const std::string& arg);


/////////////////////////////// ROS Logging ///////////////////////////////////

void log_info (Command*, AdapterExecInterface*);
void log_warning (Command*, AdapterExecInterface*);
void log_error (Command*, AdapterExecInterface*);
void log_debug (Command*, AdapterExecInterface*);

#endif
