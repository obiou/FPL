#ifndef CFACTORY_H
#define CFACTORY_H

#include <iostream>
#include <map>
#include <string>

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
        static void Delete() {
            delete Factory<Interface,StaticInterface>::m_pInstance;
            Factory<Interface,StaticInterface>::m_pInstance = NULL;
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
#ifdef DEBUG
            std::cout << "Registering: " << Derived::sRegistrationName << std::endl;
#endif
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
            // Delete the registered static classes
            typename MakeStaticPtrMap::iterator it;
            for( it = m_MakeStaticPtrMap.begin(); 
                 it != m_MakeStaticPtrMap.end(); it++ ) {
                delete it->second;
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
        ~RegisterClass() { Factory<Interface,StaticInterface>::Delete(); }
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
    /// This static variable will trigger automatic registration
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
    class INTERFACE_NAME :                                              \
    public GENERIC_FACTORY_NAMESPACE::Interface_CRTP< INTERFACE_NAME<GENERIC_FACTORY_NAMESPACE::EmptyObject>, Derived > { \
    public:                                                             \
        static std::string sRegistrationName;                           \
        std::string GetName() { return sRegistrationName; }             \
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
    template<class> class INTERFACE_NAME##StaticImpl;                   \
    template< class Derived >                                           \
    class INTERFACE_NAME :                                              \
    public GENERIC_FACTORY_NAMESPACE::Interface_CRTP< INTERFACE_NAME<GENERIC_FACTORY_NAMESPACE::EmptyObject>, Derived, \
        INTERFACE_NAME##Static,                                         \
        INTERFACE_NAME##StaticImpl<Derived> > {                         \
public:                                                                 \
        static std::string sRegistrationName;                           \
        std::string GetName() { return sRegistrationName; }             \
    };                                                                  \
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
#endif

