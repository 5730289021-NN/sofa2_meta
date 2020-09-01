#pragma once
#include <std_srvs/SetBool.h>

namespace base_operator
{
    class Operation
    {
    public:
        Operation(){}
        virtual ~Operation(){}
        virtual void operate() = 0;
        virtual void stop() = 0;
        virtual bool serviceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) = 0;
    };
} // namespace base_operator