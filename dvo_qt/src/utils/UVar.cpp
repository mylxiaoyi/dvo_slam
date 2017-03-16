#include "UVar.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <fnmatch.h>

#include "UCommand.h"

using namespace std;

std::list<BaseMap*> UVar::maps;

UVar uvar;

UVar::UVar ()
{
}

void UVar::LoadFile (string sFileName)
{
    ifstream ifs;

    vector<string> v = ChopAndUnquoteString (sFileName);

    if (v.size () < 1) return;

    ifs.open (v[0].c_str ());

    if (!ifs.good ())
    {
        cerr << "! GUI_impl::Loadfile: Failed to load script file \""
             << sFileName << "\"." << endl;
        return;
    }

    ParseStream (ifs);
    ifs.close ();
}

void UVar::ParseStream (istream& is)
{
    string buffer;
    while (getline (is, buffer))
    {
        // Lines ending with '\' are taken as continuing on the next line.
        while (!buffer.empty () && buffer[buffer.length () - 1] == '\\')
        {
            string buffer2;
            if (!getline (is, buffer2)) break;
            buffer = buffer.substr (0, buffer.length () - 1) + buffer2;
        }
        ParseLine (buffer);
    }
}

void UVar::ParseLine (string s, bool bSilentFailure)
{
    s = UncommentString (s);
    if (s == "") return;


    // New for 2004! brace expansion! any line with {gvarname} in it
    // will have the brace bit replaced with the contents of that gvar,
    // one line per word in the gvar value...
    // e.g.
    // > TrackerList = Tracker FireTracker
    // > {TrackerList}.ReloadModel
    // would be equivalent to writing
    // > Tracker.ReloadModel
    // > FireTracker.ReloadModel
    // Can use double braces so that brace expansion only occurs later;
    // e.g. GUI_Motif.AddPushButton DoStuffToTrackerList
    // {{TrackerList}}.do_stuff
    // will create a single button which then exectues {TrackerList}.do_stuff
    // (Without double braces, the it would try to make two buttons!)

    //    { // Brace expansion wrapper follows:
    //        string::size_type nOpenBracePos = s.find ("{");
    //        // int nCloseBracePos = s.rfind("}");
    //        string::size_type nCloseBracePos = FindCloseBrace (s,
    //        nOpenBracePos, '{', '}');
    //        if ((nOpenBracePos != s.npos) && (nCloseBracePos != s.npos) &&
    //            (nCloseBracePos > nOpenBracePos))
    //        { // Brace Pair found!!

    //            string sBegin = s.substr (0, nOpenBracePos);
    //            string sVarName =
    //            s.substr (nOpenBracePos + 1, nCloseBracePos - nOpenBracePos -
    //            1);
    //            string sEnd = s.substr (nCloseBracePos + 1);

    //            string::size_type nLength =
    //            sVarName.size (); // Check if it's a double brace: {{foo}} in
    //            which
    //                              // case remove one pair, but don't expand.
    //            bool bIsDoubleQuoted = false;

    //            if (nLength > 2)
    //                if ((sVarName[0] == '{') && (sVarName[nLength - 1] ==
    //                '}'))
    //                    bIsDoubleQuoted = true;

    //            if (!bIsDoubleQuoted)
    //            {
    //                // vector<string> vsExpandedList =
    //                // ChopAndUnquoteString(mpGV2->StringValue(sVarName,
    //                true));
    //                vector<string> vsExpandedList =
    //                ChopAndUnquoteString (GV3::get_var (sVarName));

    //                // ER: look at the end of the string for '$'s to also
    //                perform
    //                // the substitution
    //                vector<string> chopped;
    //                string end;
    //                string::size_type dollarpos = s.npos, lastpos = 0;
    //                while ((dollarpos = sEnd.find ("$", dollarpos + 1)) !=
    //                s.npos)
    //                {
    //                    chopped.push_back (sEnd.substr (lastpos, dollarpos -
    //                    lastpos));
    //                    lastpos = dollarpos + 1;
    //                }
    //                end = sEnd.substr (lastpos);

    //                for (vector<string>::iterator i = vsExpandedList.begin ();
    //                     i != vsExpandedList.end (); i++)
    //                {
    //                    string line = sBegin + *i;
    //                    for (vector<string>::iterator s = chopped.begin ();
    //                         s != chopped.end (); s++)
    //                        line += *s + *i;
    //                    line += end;

    //                    ParseLine (line, bSilentFailure);
    //                }
    //                return;
    //            };

    //            // if it was double quoted, just parse it as normal, but
    //            replace s
    //            // with the new, single-braced thingy.
    //            s = sBegin + sVarName + sEnd;
    //        }
    //    }

    //    // Newer for 2004: Round brace expansion
    //    // Expands gvar in round braces to the value
    //    // e.g.  A = 2
    //    //       B = (A)
    //    // assigns 2 to B
    //    // And as before, use double brace to protect.

    //    { // Round expansion wrapper follows:

    //        string::size_type nOpenBracePos = s.find ("(");
    //        // int nCloseBracePos = s.rfind(")");
    //        string::size_type nCloseBracePos = FindCloseBrace (s,
    //        nOpenBracePos, '(', ')');

    //        //		cerr << "((( " << nOpenBracePos << "  " <<
    //        nCloseBracePos
    //        <<
    //        //endl;


    //        if ((nOpenBracePos != s.npos) && (nCloseBracePos != s.npos) &&
    //            (nCloseBracePos > nOpenBracePos))
    //        { // Brace Pair found!!
    //            // cerr << "Found (\n";
    //            // cout << "Found brace pair. " << endl;
    //            string sBegin = s.substr (0, nOpenBracePos);
    //            string sVarName =
    //            s.substr (nOpenBracePos + 1, nCloseBracePos - nOpenBracePos -
    //            1);
    //            string sEnd = s.substr (nCloseBracePos + 1);

    //            // cerr << "varname = --" << sVarName << "--\n";

    //            string::size_type nLength =
    //            sVarName.size (); // Check if it's a double brace: {{foo}} in
    //            which
    //                              // case remove one pair, but don't expand.
    //            bool bIsDoubleQuoted = false;

    //            if (nLength > 2)
    //                if ((sVarName[0] == '(') && (sVarName[nLength - 1] ==
    //                ')'))
    //                {
    //                    s = sBegin + sVarName +
    //                        sEnd; // Just remove the first set of braces.
    //                    bIsDoubleQuoted = true;
    //                };

    //            if (!bIsDoubleQuoted)
    //            {
    //                // cerr << "varname = --" << sVarName << "--\n";
    //                // cerr << "***************" << GV3::get_var(sVarName) <<
    //                endl;
    //                // string sExpanded = mpGV2->StringValue(sVarName, true);
    //                string sExpanded = GV3::get_var (sVarName);
    //                s = sBegin + sExpanded + sEnd;
    //                // cout << "DEBUG : xx" << s << "xx" << endl;
    //            };
    //        }
    //    }


    // Old ParseLine code follows, here no braces can be left (unless in arg.)
    istringstream ist (s);

    string sCommand;
    string sParams;

    // Get the first token (the command name)
    ist >> sCommand;
    if (sCommand == "") return;

    // Get everything else (the arguments)...

    // Remove any whitespace
    ist >> ws;
    getline (ist, sParams);

    // Attempt to execute command
    if (ucommand.Call (sCommand, sParams)) return;

    if (setvar (s)) return;

    if (!bSilentFailure)
        cerr << "? GUI_impl::ParseLine: Unknown command \"" << sCommand
             << "\" or invalid assignment." << endl;
}

bool UVar::setvar (string s)
{
    // Execution failed. Maybe its an assignment.
    string::size_type n;
    n = s.find ("=");

    if (n != string::npos)
    {
        string var = s.substr (0, n);
        string val = s.substr (n + 1);

        // Strip whitespace from around var;
        string::size_type s = 0, e = var.length () - 1;
        for (; isspace (var[s]) && s < var.length (); s++)
        {
        }
        if (s == var.length ()) // All whitespace before the `='?
            return false;
        for (; isspace (var[e]); e--)
        {
        }
        if (e >= s)
        {
            var = var.substr (s, e - s + 1);

            // Strip whitespace from around val;
            s = 0, e = val.length () - 1;
            for (; isspace (val[s]) && s < val.length (); s++)
            {
            }
            if (s < val.length ())
            {
                for (; isspace (val[e]); e--)
                {
                }
                val = val.substr (s, e - s + 1);
            }
            else
                val = "";

            set_var (var, val);
            return true;
        }
    }

    return false;
}

string UVar::get_var (string name)
{
    if (registered_type_and_trait.count (name))
        return registered_type_and_trait[name].first->get_as_string (name, 0);
    else if (unmatched_tags.count (name))
        return unmatched_tags[name];
    else
        return "(Not present in GVar list.)";
}

bool UVar::set_var (string name, string val, bool silent)
{
    if (registered_type_and_trait.count (name))
    {
        int e = registered_type_and_trait[name].first->set_from_string (name, val);
        if (!silent)
            parse_warning (e, registered_type_and_trait[name].first->name (), name, val);
        return e == 0;
    }
    else
    {
        unmatched_tags[name] = val;
        return true;
    }
}

void UVar::parse_warning (int e, string type, string name, string from)
{
    if (e > 0)
        std::cerr << "! GV3:Parse error setting " << type << " " << name
                  << " from " << from << std::endl;
    else if (e < 0)
        std::cerr << "! GV3:Parse warning setting " << type << " " << name
                  << " from " << from << ": "
                  << "junk is -->" << from.c_str () - e << "<--" << std::endl;
}

vector<string> UVar::ChopAndUnquoteString (string s)
{
    vector<string> v;
    string::size_type nPos = 0;
    string::size_type nLength = s.length ();
    while (1)
    {
        string sTarget;
        char cDelim;
        // Get rid of leading whitespace:
        while ((nPos < nLength) && (s[nPos] == ' ')) nPos++;
        if (nPos == nLength) return v;

        // First non-whitespace char...
        if (s[nPos] != '\"')
            cDelim = ' ';
        else
        {
            cDelim = '\"';
            nPos++;
        }
        for (; nPos < nLength; ++nPos)
        {
            char c = s[nPos];
            if (c == cDelim) break;
            if (cDelim == '"' && nPos + 1 < nLength && c == '\\')
            {
                char escaped = s[++nPos];
                switch (escaped)
                {
                case 'n':
                    c = '\n';
                    break;
                case 'r':
                    c = '\r';
                    break;
                case 't':
                    c = '\t';
                    break;
                default:
                    c = escaped;
                    break;
                }
            }
            sTarget += c;
        }
        v.push_back (sTarget);

        if (cDelim == '\"') nPos++;
    }
}

string UVar::UncommentString (string s)
{
    // int n = s.find("//");
    // return s.substr(0,n);

    int q = 0;

    for (string::size_type n = 0; n < s.size (); n++)
    {
        if (s[n] == '"') q = !q;

        if (s[n] == '/' && !q)
        {
            if (n < s.size () - 1 && s[n + 1] == '/') return s.substr (0, n);
        }
    }

    return s;
}

void UVar::add_typemap(BaseMap *m)
{
    maps.push_back(m);
}

void UVar::print_var_list (ostream& o, string pattern, bool show_all)
{
    bool no_pattern = (pattern == "");

    if (show_all) o << "//Registered GVars:" << endl;

    for (map<string, std::pair<BaseMap*, int>>::iterator i =
         registered_type_and_trait.begin ();
         i != registered_type_and_trait.end (); i++)
        if (show_all || !(i->second.second & HIDDEN))
            if (no_pattern || !fnmatch (pattern.c_str (), i->first.c_str (), FNM_CASEFOLD))
                o << i->first << "="
                  << i->second.first->get_as_string (i->first, 1) << endl;

    if (show_all)
    {
        o << "//Unmatched tags:" << endl;

        for (map<string, string>::iterator i = unmatched_tags.begin ();
             i != unmatched_tags.end (); i++)
            if (no_pattern || !fnmatch (pattern.c_str (), i->first.c_str (), FNM_CASEFOLD))
                o << i->first << "=" << i->second << endl;

        o << "// End of GVar list." << endl;
    }
}

vector<string> UVar::tag_list ()
{
    vector<string> v;
    for (map<string, std::pair<BaseMap*, int>>::iterator i =
         registered_type_and_trait.begin ();
         i != registered_type_and_trait.end (); i++)
        v.push_back (i->first);

    return v;
}

vector<string> UVar::all_tag_list ()
{
    vector<string> v;
    for (map<string, std::pair<BaseMap*, int>>::iterator i =
         registered_type_and_trait.begin ();
         i != registered_type_and_trait.end (); i++)
        v.push_back (i->first);

    for (map<string, string>::iterator i = unmatched_tags.begin ();
         i != unmatched_tags.end (); i++)
        v.push_back (i->first);

    return v;
}
