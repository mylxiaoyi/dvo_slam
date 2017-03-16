#ifndef UCOMMAND_H
#define UCOMMAND_H

#include <string>
#include <map>
#include <set>
#include <vector>

class UCommand;

extern UCommand ucommand;

typedef void (*CallbackProc) (void* ptr, std::string sCommand, std::string sParams);

typedef struct
{
    CallbackProc cbp;
    void* thisptr;
} CallbackInfoStruct;

typedef std::vector<CallbackInfoStruct> CallbackVector;

class UCommand
{
public:
    UCommand();

    void RegisterCommand (std::string sCommandName, CallbackProc callback, void* thisptr = NULL);
    void UnRegisterAllCommands (void* thisptr);
    void UnRegisterCommand (std::string sCommandName, void* thisptr);
    void UnRegisterCommand (std::string sCommandName);

    bool Call (std::string sCommand, std::string sParams);

private:

    void do_builtins ();
    void RegisterBuiltin (std::string sCommandName, CallbackProc callback);

    std::map<std::string, CallbackVector> mmCallBackMap;
    std::set<std::string> builtins;
};

#endif // UCOMMAND_H
