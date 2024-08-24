// Singleton design pattern
// Singleton class is a class that can have only one object (an instance of the class) at a time.
#pragma once

class Singleton {
   protected:
    // Private constructor to prevent instantiation
    Singleton() {}

    // Delete copy constructor and assignment operator
    Singleton(const Singleton&) = delete;
    Singleton& operator=(const Singleton&) = delete;

   public:
    // Static method to get the instance of the class
    static Singleton& getInstance() {
        static Singleton instance;  // Guaranteed to be destroyed and instantiated on first use
        return instance;
    }
};