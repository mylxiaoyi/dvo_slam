#ifndef UVAR_H
#define UVAR_H

#include <iostream>
#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "Serialize.h"
#include "default.h"
#include "type_name.h"

class UVar;

extern UVar uvar;

struct type_mismatch
{
};

struct gvar_was_not_defined
{
};

// Bit-masks for gvar registration:
// SILENT makes gvars not complain if it has to use the default;
// HIDDEN makes vars not appear in gvarlist unless used with -a

enum
{
    SILENT = 1 << 0,
    HIDDEN = 1 << 1,
    FATAL_IF_NOT_DEFINED = 1 << 2
};

class BaseMap
{
public:
    virtual std::string get_as_string (const std::string& name, bool precise) = 0;
    virtual int set_from_string (const std::string& name, const std::string& val) = 0;
    virtual std::string name () = 0;
    virtual std::vector<std::string> list_tags () = 0;
    virtual ~BaseMap (){};
};

class UVar
{
private:
    template <class T> class TypedMap : public BaseMap
    {
    private:
        friend class UVar;

        // This gives us singletons
        static TypedMap& instance ()
        {
            static TypedMap* inst = 0;

            if (!inst)
            {
                inst = new TypedMap ();
                // Register ourselves with GV3
                UVar::add_typemap (inst);
            }

            return *inst;
        }

        // Get a data member
        ValueHolder<T>* get (const std::string& n)
        {
            DataIter i;

            i = data.find (n);

            if (i == data.end ())
                return NULL;
            else
                return &(i->second);
        }

        ValueHolder<T>* safe_replace (const std::string& n, const T& t)
        {
            DataIter i, j;
            // Keep track of the neighboring point
            // to pass as a hint to insert.
            i = data.find (n);

            if (i == data.end ())
            {
                return &(data.insert (make_pair (n, t)).first->second);
            }
            else
            {
                i->second.set (t);
                return &(i->second);
            }
        }

        // Create a data member
        ValueHolder<T>* create (const std::string& n)
        {
            return &(data.insert (make_pair (n, DefaultValue<T>::val ()))->second);
        }

        virtual int set_from_string (const std::string& name, const std::string& val)
        {
            std::istringstream is (val);
            T tmp = serialize::from_stream<T> (is);
            int e = serialize::check_stream (is);

            if (e == 0) safe_replace (name, tmp);
            return e;
        }

        virtual std::string get_as_string (const std::string& name, bool precise)
        {
            DataIter i = data.find (name);

            if (i == data.end ())
                i = data.insert (make_pair (name, DefaultValue<T>::val ())).first;

            return serialize::to_string (i->second.get (), precise);
        }

        virtual std::string name ()
        {
            return type_name<T> ();
        }

        virtual std::vector<std::string> list_tags ()
        {
            std::vector<std::string> l;
            for (DataIter i = data.begin (); i != data.end (); i++)
                l.push_back (i->first);
            return l;
        }

        std::map<std::string, ValueHolder<T>> data;
        typedef typename std::map<std::string, ValueHolder<T>>::iterator DataIter;
    };

    template <class T> friend class TypedMap;

    template <class T> ValueHolder<T>* attempt_get (const std::string& name)
    {
        ValueHolder<T>* d = TypedMap<T>::instance ().get (name);

        if (!d) // Data not present in map of the correct type
        {
            // Does it exist with a different type?
            if (registered_type_and_trait.count (name))
            { // Yes: programmer error.
                std::cerr << "GV3:Error: type mismatch while getting "
                          << type_name<T> () << " " << name
                          << ": already registered "
                             "as type "
                          << registered_type_and_trait[name].first->name ()
                          << ". Fix your code.\n";

                throw type_mismatch ();
            }
            else
                return NULL;
        }

        return d;
    }

    template <class T>
    static ValueHolder<T>* safe_replace (const std::string& name, const T& t)
    {
        return TypedMap<T>::instance ().safe_replace (name, t);
    }

    static void add_typemap (BaseMap* m);

    std::map<std::string, std::string> unmatched_tags;
    std::map<std::string, std::pair<BaseMap*, int>> registered_type_and_trait;
    static std::list<BaseMap*> maps;

    template <class T>
    ValueHolder<T>* get_by_val (const std::string& name, const T& default_val, int flags);
    template <class T>
    ValueHolder<T>*
    get_by_str (const std::string& name, const std::string& default_val, int flags);
    template <class T>
    ValueHolder<T>*
    register_new_gvar (const std::string& name, const T& default_val, int flags);

    void parse_warning (int e, std::string type, std::string name, std::string from);

public:
    UVar ();

    void ParseLine (std::string s, bool bSilentFailure = false);
    void ParseStream (std::istream& is);
    void LoadFile (std::string sFileName);

    // Get and set by string only
    std::string get_var (std::string name);
    bool set_var (std::string name, std::string val, bool silent = false);

    // Get references by name
    template <class T>
    T& get (const std::string& name, const T& default_val = defaultValue<T> (), int flags = 0);
    template <class T>
    T& get (const std::string& name, std::string default_val, int flags = 0);

    // Some helper functions
    void print_var_list (std::ostream& o, std::string pattern = "", bool show_all = true);
    std::vector<std::string> tag_list ();
    std::vector<std::string> all_tag_list ();

private:

    std::vector<std::string> ChopAndUnquoteString (std::string s);
    std::string UncommentString (std::string s);
    bool setvar (std::string s);
};

#include "UVar_implementation.hh"

#endif // UVAR_H
