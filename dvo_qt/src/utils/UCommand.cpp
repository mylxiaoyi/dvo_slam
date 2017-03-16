#include "UCommand.h"
#include "UVar.h"

#include <iostream>

using namespace std;

UCommand ucommand;

void buildInHandler(void *ptr, std::string sCommand, std::string sParams)
{
    UVar *uvar_ptr = (UVar*)ptr;
    if (sCommand == "include")
        uvar_ptr->LoadFile(sParams);
}

UCommand::UCommand()
{
    do_builtins();
}

void UCommand::do_builtins ()
{
    RegisterCommand("include", buildInHandler, &uvar);
}

void UCommand::RegisterBuiltin (std::string sCommandName, CallbackProc callback)
{
    RegisterCommand (sCommandName, callback, this);
    builtins.insert (sCommandName);
}

void UCommand::RegisterCommand (std::string sCommandName, CallbackProc callback, void* thisptr)
{
    if (builtins.count (sCommandName))
    {
        cerr
        << "!!GUI_impl::RegisterCommand: Tried to register reserved keyword "
        << sCommandName << "." << endl;
        return;
    }

    CallbackInfoStruct s;
    s.cbp = callback;
    s.thisptr = thisptr;

    bool bAlreadyThere = false;
    CallbackVector* cbv = &mmCallBackMap[sCommandName];


    for (CallbackVector::iterator i = cbv->begin (); i < cbv->end (); i++)
        if ((i->cbp == s.cbp) && (i->thisptr == s.thisptr))
            bAlreadyThere = true;

    if (!bAlreadyThere) cbv->push_back (s);
}

void UCommand::UnRegisterCommand (string sCommandName)
{
    mmCallBackMap.erase (sCommandName);
}

void UCommand::UnRegisterCommand (string sCommandName, void* thisptr)
{
    CallbackVector& cbv = mmCallBackMap[sCommandName];
    for (int i = static_cast<int> (cbv.size ()) - 1; i >= 0; i--)
        if (cbv[i].thisptr == thisptr) cbv.erase (cbv.begin () + i);
}

void UCommand::UnRegisterAllCommands (void* thisptr)
{
    for (map<string, CallbackVector>::iterator i = mmCallBackMap.begin ();
         i != mmCallBackMap.end (); i++)
        UnRegisterCommand (i->first, thisptr);
}

bool UCommand::Call (string sCommand, string sParams)
{
    if (mmCallBackMap.count (sCommand) == 0) return false;

    // Make a copy of this callback vector, since the command might call
    // Unregister.
    CallbackVector cbv = mmCallBackMap[sCommand];
    if (cbv.size () == 0) return false;

    for (CallbackVector::iterator i = cbv.begin (); i < cbv.end (); i++)
        i->cbp (i->thisptr, sCommand, sParams);

    return true;
}
