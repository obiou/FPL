#include <stdio.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <map>
#include <string>

#define GENERIC_FACTORY_NAMESPACE MyFactory

////////////////////////////////////////////////////////////////////////////////
/// Generic Factory with automatic registration through inheritance.
///
/// How does it work?  
///
/// In this context, we will call 'Programmer' the person who designs
/// an interface that will define the objects that will be created by
/// the Factory. 'User' will refer to the person who writes the
/// objects (or 'plugins') that the Factory should be able to automatically
/// create.
///
/// The factory is templated on a 'Interface' class and a
/// 'StaticInterface' class that enables calls to static functions of
/// derived classes. 'StaticInterface' can be ignored if no static calls on the
/// objects are required.
///
/// For automatic registration, derived classes should inherit from
/// Interface_CRTP< Interface, StaticInterface, Derived,
/// StaticInterfaceImpl<Derived> >.  (It is recommended to write a
/// proxy or use the macros to simplify the final User's code.)
/// 
/// The User's derived classes are required to contain a static
/// string 'sRegistrationName' that will correspond to the name under which the
/// derived class instantiation will be registered (i.e. used by 'Create')
///
/// CRTP is used to automatically provide a 'make' function for the
/// factory and a 'sRegistrationName' variable. 'sRegistrationName' is
/// added to the CRTP class by the Programmer. This imposes an
/// difficult call by the User that can be simplified by the
/// FACTORY_OBJECT macro.
namespace GENERIC_FACTORY_NAMESPACE /// This namespace is meant to be redefined
{
    class EmptyStaticInterface {};
    class EmptyStaticObject : public EmptyStaticInterface {};
    class EmptyObject {};

    template<class Interface,class StaticInterface=EmptyStaticInterface>
    class Factory {
        typedef Interface* (*MakePtr)();
        typedef std::map<std::string, MakePtr> MakePtrMap;
        typedef std::map<std::string, StaticInterface*> MakeStaticPtrMap;
    
    public:
        ////////////////////////////////////////////////////////////////////////////
        /// Call to singleton
        static Factory<Interface,StaticInterface>& GetInstance() {
            if( m_pInstance == NULL ) {
                m_pInstance = new Factory<Interface,StaticInterface>();
            } 
            return *m_pInstance;
        }

        ////////////////////////////////////////////////////////////////////////////
        /// Call to create instance
        static Interface* Create( const std::string& sType ) {
            return GetInstance()._Create( sType );
        }

        ////////////////////////////////////////////////////////////////////////////
        /// Call to obtain registered static part
        static StaticInterface* Static( const std::string& sType ) {
            return GetInstance()._Static( sType );
        }

        ////////////////////////////////////////////////////////////////////////////
        template<class Derived>
        bool Register() {
            if( m_MakePtrMap.find( Derived::sRegistrationName ) != m_MakePtrMap.end() ) {
                std::cerr << "WARNING: in Factory, a '" << Derived::sRegistrationName
                          << "' is already registered, overwriting." << std::endl;
            }
            m_MakePtrMap[ Derived::sRegistrationName ] = Derived::make;
            return true;
        }

        ////////////////////////////////////////////////////////////////////////////
        template<class Derived>
        bool RegisterStatic() { 
            if( m_MakeStaticPtrMap.find( Derived::sRegistrationName ) != m_MakeStaticPtrMap.end() ) {
                std::cerr << "WARNING: in Factory, a '" << Derived::sRegistrationName
                          << "' static object is already registered, overwriting." << std::endl;
                StaticInterface* pI = m_MakeStaticPtrMap.find( Derived::sRegistrationName )->second;
                m_MakeStaticPtrMap[ Derived::sRegistrationName ] = NULL;
                delete pI;
            }
            // Call the make function to generate a static class
            m_MakeStaticPtrMap[ Derived::sRegistrationName ] = Derived::makeStatic();
            return true;
        }

        ////////////////////////////////////////////////////////////////////////////
        template<class Function>
        void ForEach( Function f ) {
            typename std::map<std::string,MakePtr>::iterator it;
            for( it = m_MakePtrMap.begin(); it != m_MakePtrMap.end(); it++ ) {
                f( it->first );
            }
        }

        ////////////////////////////////////////////////////////////////////////////
        void PrintRegistered() {
            ForEach( std::ostream_iterator<int>( std::cout, "\n") );
        }

    private:
        ////////////////////////////////////////////////////////////////////////////
        Interface* _Create( const std::string& sType ) {
            if( m_MakePtrMap.find( sType ) == m_MakePtrMap.end() ) {
                std::cerr << "ERROR: in Factory, '" << sType 
                          << "' unknown." << std::endl;
                return NULL;
            }
            return m_MakePtrMap[ sType ]();
        }

        ////////////////////////////////////////////////////////////////////////////
        StaticInterface* _Static( const std::string& sType ) {
            if( m_MakePtrMap.find( sType ) == m_MakePtrMap.end() ) {
                std::cerr << "ERROR: in Factory Static, '" << sType 
                          << "' unknown as static function." << std::endl;
                return NULL;
            }
            return m_MakeStaticPtrMap[ sType ];
        }

    private:
        ////////////////////////////////////////////////////////////////////////////
        Factory() {}
        ~Factory() {
            if( m_pInstance != NULL ) { delete m_pInstance; }
            m_pInstance = NULL; 
            // Delete the registered static classes
            typename MakeStaticPtrMap::iterator it;
            for( it = m_MakeStaticPtrMap.begin(); 
                 it != m_MakeStaticPtrMap.end(); it++ ) {
                delete it;
            }
            m_MakeStaticPtrMap.clear();
        }
  
    private:
        MakePtrMap m_MakePtrMap;
        MakeStaticPtrMap m_MakeStaticPtrMap;
        static Factory<Interface,StaticInterface>* m_pInstance;
    };

    ////////////////////////////////////////////////////////////////////////////////
    template<class Interface, class StaticInterface>
    Factory<Interface,StaticInterface>* Factory<Interface,StaticInterface>::m_pInstance = NULL;

    ////////////////////////////////////////////////////////////////////////////////
    /// This class is used to automatically register calls in the factory
    template<class Interface, class Derived, class StaticInterface>
    class RegisterClass {
    public:
        RegisterClass() { // dodgy C++ syntax, must explicitly say it's a template
            Factory<Interface,StaticInterface>::GetInstance().
                Factory<Interface,StaticInterface>::template Register<Derived>();
            Factory<Interface,StaticInterface>::GetInstance().
                Factory<Interface,StaticInterface>::template RegisterStatic<Derived>();
        }
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// CRTP is used to automatically generate a 'make' and 'makeStatic' call.
    template<class Interface, class Derived,
             class StaticInterface=EmptyStaticInterface, 
             class DerivedStatic=EmptyStaticObject>
    class Interface_CRTP : public Interface {
    public:
        Interface_CRTP() {
            (void)&m_Registerer; // used to ensure instantiation (useful despite warning)
        }
        static Interface* make() { return new Derived(); }
        static StaticInterface* makeStatic() { return new DerivedStatic(); }
    private:
        static RegisterClass<Interface,Derived,StaticInterface> m_Registerer;
        static int val;
    };

    ////////////////////////////////////////////////////////////////////////////////
    /// This static variable with trigger automatic registration
    template<class Interface, class Derived, 
             class StaticInterface, class DerivedStatic>
    RegisterClass<Interface,Derived,StaticInterface> 
    Interface_CRTP<Interface,Derived,StaticInterface,DerivedStatic>::m_Registerer;
}
/// Internal macro, not to be used directly
#define FACTORY_INTERFACE_DECL( INTERFACE_NAME ) \
    template<> \
    class INTERFACE_NAME<GENERIC_FACTORY_NAMESPACE::EmptyObject>

/// This macro should be used to define a macro where the interface has no static
/// functions (or you wish to make static calls after a class creation.)
/// It creates a typedef INTERFACE_NAMEFactory to ease calls 
/// to the factory for the specific objects implementing the interface.
#define FACTORY_INTERFACE( INTERFACE_NAME ) \
    template< class Derived = GENERIC_FACTORY_NAMESPACE::EmptyObject > \
    class INTERFACE_NAME;                                               \
    typedef GENERIC_FACTORY_NAMESPACE::Factory<INTERFACE_NAME<> > INTERFACE_NAME##Factory; \
    template< class Derived > \
    class INTERFACE_NAME : \
        public GENERIC_FACTORY_NAMESPACE::Interface_CRTP< INTERFACE_NAME<GENERIC_FACTORY_NAMESPACE::EmptyObject>, Derived > { \
    public:                                                             \
        static std::string sRegistrationName;                                       \
    };                                                                  \
    FACTORY_INTERFACE_DECL( INTERFACE_NAME )

/// This macro should be used to define an interface with static
/// functions.
/// It creates a typedef INTERFACE_NAMEFactory to ease calls 
/// to the factory for the specific objects implementing the interface.
/// Three more declarations should then be made:
/// - FACTORY_INTERFACE_STATIC should be called to define the
///   interface class with ONLY the static functions.
/// - FACTORY_INTERFACE_STATIC_IMPL should be called to define an
///   implementation of the static interface where the static calls
///   have me replaced with member calls to the static calls.
#define FACTORY_INTERFACE_WITH_STATIC( INTERFACE_NAME )    \
    template< class Derived = GENERIC_FACTORY_NAMESPACE::EmptyObject > class INTERFACE_NAME; \
    class INTERFACE_NAME##Static;                                   \
    typedef GENERIC_FACTORY_NAMESPACE::Factory<INTERFACE_NAME<>,INTERFACE_NAME##Static> INTERFACE_NAME##Factory; \
    template<class> class INTERFACE_NAME##StaticImpl;               \
    template< class Derived >                                           \
    class INTERFACE_NAME : \
        public GENERIC_FACTORY_NAMESPACE::Interface_CRTP< INTERFACE_NAME<GENERIC_FACTORY_NAMESPACE::EmptyObject>, Derived, \
                               INTERFACE_NAME##Static,                  \
                               INTERFACE_NAME##StaticImpl<Derived> > { \
    public:                                                            \
        static std::string sRegistrationName;                                      \
    };                                                                 \
    FACTORY_INTERFACE_DECL( INTERFACE_NAME )

#define FACTORY_INTERFACE_ONLY_STATIC( INTERFACE_NAME ) \
    class INTERFACE_NAME##Static

#define FACTORY_INTERFACE_ONLY_STATIC_IMPL( INTERFACE_NAME ) \
    template<class Derived> \
    class INTERFACE_NAME##StaticImpl : public INTERFACE_NAME##Static

#define FACTORY_OBJECT( OBJECT_NAME, INTERFACE_NAME, REGISTERATION_NAME ) \
    class OBJECT_NAME;                                                  \
    template<>                                                          \
    std::string INTERFACE_NAME<OBJECT_NAME>::sRegistrationName = REGISTERATION_NAME; \
    class OBJECT_NAME : public INTERFACE_NAME< OBJECT_NAME > 

// End of generic code
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// The following code shows an example of how to generate interfaces
/// that will be implemented by the user when he wishes to add derived
/// classes ("plugins") to the Factory.
///
/// In this example, we assume the Interface does not contain any static
/// functions.  We use a recursive template to ensure the Interface and
/// Proxy have same name which should be clearer for the final
/// User. The equivalent (and alternative solution) is: 
///   - Programmer code:
///   class MyInterface1{ ...code ... }; 
///   class MyInterface1Proxy : public Interface_CRTP< MyInterface1, Derived > {}; 
///   - User code:
///   class MyDerived11 : MyInterface1Proxy< MyDerived11 > { ... code ... };
///
////////////////////////////////////////////////////////////////////////////////
/// Programmer code
template< class Derived = MyFactory::EmptyObject > class MyInterface1;
typedef MyFactory::Factory<MyInterface1<> > MyInterface1Factory;

////////////////////////////////////////////////////////////////////////////////
/// Programmer code
/// This proxy class is there to simplify the client calls that then only
/// needs to inherit from MyInterface1< Derived >.
template< class Derived >
class MyInterface1 : public MyFactory::Interface_CRTP< MyInterface1< MyFactory::EmptyObject >, Derived > {
public:
    static std::string sRegistrationName;
};

////////////////////////////////////////////////////////////////////////////////
/// Programmer code
template<>
class MyInterface1<MyFactory::EmptyObject> {
public:
    MyInterface1() { std::cout << "MyInterface1 constructor" << std::endl; }
    virtual void print() = 0;
};

////////////////////////////////////////////////////////////////////////////////
/// Programmer (or User) code
class FactoryUser1 : public MyInterface1<> {
public:
    FactoryUser1() { std::cout << "FactoryUser1 constructor" << std::endl; }

    void print() {
        std::cout << "Print" << std::endl;
    }

    void print( std::string sType ) {
        MyInterface1<> *a = MyFactory::Factory<MyInterface1<> >::Create( sType );
        if( a != NULL) { a->print(); delete a; }
    }
};

////////////////////////////////////////////////////////////////////////////////
/// User code
class MyDerived11 : public MyInterface1< MyDerived11 > {
public:
    MyDerived11() { std::cout << "MyDerived11 constructor" << std::endl; }
    void print() { std::cout << ">>> MyDerived11 print." << std::endl; }    
};

template<>
std::string MyInterface1<MyDerived11>::sRegistrationName = "MyDerived11";

////////////////////////////////////////////////////////////////////////////////
/// User code
class MyDerived12 : public MyInterface1< MyDerived12 > {
public:
    MyDerived12() { std::cout << "MyDerived12 constructor" << std::endl; }
    void print() { std::cout << ">>> MyDerived12 print." << std::endl; }
};

template<>
std::string MyInterface1<MyDerived12>::sRegistrationName = "MyDerived12";

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// The following code shows an example of how to generate interfaces
/// that will be implemented by the user when he wishes to add derived
/// classes ("plugins") to the Factory.
///
/// It is more complicated than the previous example as it provides the
/// possibility to access static functions without requiring to
/// create (call to 'new') a full derived class.
/// As previously, we use a recursive template to ensure coherence in naming
/// for the final user.
////////////////////////////////////////////////////////////////////////////////
/// Programmer code
template< class Derived = MyFactory::EmptyObject > class MyInterface2;

////////////////////////////////////////////////////////////////////////////////
/// Programmer code
/// This proxy class is there to simplify the User calls that only
/// need to inherit from MyInterface2< Derived >.
class MyStaticInterface2;
template<class> class MyStaticInterface2Impl; 
typedef MyFactory::Factory<MyInterface2<>,MyStaticInterface2> MyInterface2Factory;
template< class Derived >
class MyInterface2 : 
    public MyFactory::Interface_CRTP< MyInterface2<MyFactory::EmptyObject>, Derived, 
                                      MyStaticInterface2, MyStaticInterface2Impl<Derived> > {
public:
    static std::string sRegistrationName;
};

////////////////////////////////////////////////////////////////////////////////
/// Programmer code
template<>
class MyInterface2<MyFactory::EmptyObject> {
public:
    MyInterface2() { std::cout << "Interface2 constructor" << std::endl; }
    virtual void print() = 0;
    static void info();
};

////////////////////////////////////////////////////////////////////////////////
/// Programmer code
/// This static interface is required for the Factory to work on a map
/// of static classes
class MyStaticInterface2 {
public:
    virtual void info() = 0;
};

////////////////////////////////////////////////////////////////////////////////
/// Programmer code
/// This class makes the link between the static functions in 'MyStaticInterface2' and
/// a simplified class containing only static code.
/// Here we simply repeate all the static functions available in 'MyStaticInterface2'
/// with member calls.
/// We inherit from 'MyStaticInterface2' to ensure we have implemented all
/// the member functions.
template<class Derived>
class MyStaticInterface2Impl : public MyStaticInterface2 { 
public:
    void info() { Derived::info(); }
};

////////////////////////////////////////////////////////////////////////////////
/// Programmer (or User) code
class FactoryUser2 : public MyInterface2<> {
public:
    FactoryUser2() { std::cout << "FactoryUser2 constructor" << std::endl; }

    void print() {
        std::cout << "Print" << std::endl;
    }

    void print( std::string sType ) {
        MyInterface2<> *a = MyInterface2Factory::Create( sType );
        if( a!= NULL ) { a->print(); delete a; }
    }

    static void info( std::string sType ) {
        MyStaticInterface2* pStatic = MyInterface2Factory::Static( sType );
        if( pStatic != NULL ) pStatic->info();
    }

    static void info() { std::cout << "Info." << std::endl; }
};

////////////////////////////////////////////////////////////////////////////////
/// User code
class MyDerived21 : public MyInterface2< MyDerived21 > {
public:
    MyDerived21() { std::cout << "Derived21 constructor" << std::endl; }
    void print() { std::cout << ">>> Derived21 print" << std::endl; }    
    static void info() { std::cout << ">>> Derived21 info" << std::endl; }
};

template<>
std::string MyInterface2<MyDerived21>::sRegistrationName = "MyDerived21";

////////////////////////////////////////////////////////////////////////////////
/// User code
class MyDerived22 : public MyInterface2< MyDerived22 > {
public:
    MyDerived22() { std::cout << "Derived22 constructor" << std::endl; }
    void print() { std::cout << ">>> Derived22 print" << std::endl; }
    static void info() { std::cout << ">>> Derived22 info" << std::endl; }
};

template<>
std::string MyInterface2<MyDerived22>::sRegistrationName = "MyDerived22";

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/// The following code is the same as the previous code but we use
/// macros to simplify the Programmer's code.
////////////////////////////////////////////////////////////////////////////////
/// Programmer code
/// Example of interface with no static calls
FACTORY_INTERFACE( MyInterface3 ) {
 public:
    MyInterface3() { std::cout << "MyInterface3 constructor" << std::endl; }
    virtual void print() = 0;
};

////////////////////////////////////////////////////////////////////////////////
// User code
class MyDerived31 : public MyInterface3< MyDerived31 > {
public:
    MyDerived31() { std::cout << "MyDerived31 constructor" << std::endl; }
    void print() { std::cout << ">>> MyDerived31 print." << std::endl; }    
};

template<>
std::string MyInterface3<MyDerived31>::sRegistrationName = "MyDerived31";

// Alternative call using macro (simplifies registration name definition)
FACTORY_OBJECT( MyDerived32, MyInterface3, "MyInterface32" ) {
public:
    MyDerived32() { std::cout << "MyDerived32 constructor" << std::endl; }
    void print() { std::cout << ">>> MyDerived32 print." << std::endl; }
};

////////////////////////////////////////////////////////////////////////////////
/// Programmer code
/// Example of interface with static calls
FACTORY_INTERFACE_WITH_STATIC( MyInterface4 ) {
 public:
    MyInterface4() { std::cout << "MyInterface4 constructor" << std::endl; }
    virtual void print() = 0;
    static void info();
};

FACTORY_INTERFACE_ONLY_STATIC( MyInterface4 ) {
 public:
    virtual void info() = 0;
};

FACTORY_INTERFACE_ONLY_STATIC_IMPL( MyInterface4 ) {
 public:
    void info() { Derived::info(); }
};

////////////////////////////////////////////////////////////////////////////////
/// User code
class MyDerived41 : public MyInterface4< MyDerived41 > {
public:
    MyDerived41() { std::cout << "Derived41 constructor" << std::endl; }
    void print() { std::cout << ">>> Derived41 print" << std::endl; }    
    static void info() { std::cout << ">>> Derived41 info" << std::endl; }
};

template<>
std::string MyInterface4<MyDerived41>::sRegistrationName = "MyDerived41";

// Alternative call using macro (simplifies registration name definition)
FACTORY_OBJECT( MyDerived42, MyInterface4, "MyDerived42" ) {
 public:
    MyDerived42() { std::cout << "Derived42 constructor" << std::endl; }
    void print() { std::cout << ">>> Derived42 print" << std::endl; }
    static void info() { std::cout << ">>> Derived42 info" << std::endl; }
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int main()
{
    FactoryUser1 gen;
    gen.print( "MyDerived11" );
    gen.print( "MyDerived12" );
    gen.print( "Yo!" );
    std::cout << std::endl;

    FactoryUser2 gen2;
    gen2.print( "MyDerived21" );
    gen2.print( "MyDerived22" );
    gen2.print( "Yo!" );
    gen2.info(  "MyDerived21" );
    gen2.info(  "MyDerived22" );
    gen2.info(  "Yo!" );
    gen2.info();
    std::cout << std::endl;

    // Code without static member functions
    MyInterface3<> *a = MyInterface3Factory::Create( "MyDerived31" );
    if( a!= NULL ) { a->print(); delete a; }
    std::cout << std::endl;

    // Code without static
    MyInterface4<> *b = MyInterface4Factory::Create( "MyDerived41" );
    if( b!= NULL ) { b->print(); delete b; }
    MyInterface4Static* pStatic = MyInterface4Factory::Static( "MyDerived41" );
    if( pStatic != NULL ) pStatic->info();

    return 0;
}
