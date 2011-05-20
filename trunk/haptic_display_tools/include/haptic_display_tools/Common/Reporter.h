#ifndef REPORTER_H
#define REPORTER_H

#include <string>

class Reporter
{
public:
    virtual ~Reporter() { }

    virtual void start(std::string title = "Progress",
                       std::string caption = "Performing task...") = 0;
    virtual void report(int percent) = 0;
    virtual void finish() = 0;

    virtual bool cancelled() { return false; }
};

#endif // REPORTER_H
