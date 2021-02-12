#pragma once

#include <robot_interface_eki/xml/XmlWriter.h>
#include <robot_interface_eki/core/Types.h>

namespace rbt
{
class MetaCommand
{
private:
public:
    MetaCommand(const rbt::MetaCommand *base = nullptr);
    MetaCommand(float velocity, bool abort, const rbt::MetaCommand *base = nullptr);
    MetaCommand(float velocity, bool abort, const rbt::MetaCommand &base) : MetaCommand(velocity, abort, &base){};
    ~MetaCommand() {}

    float velocity_override = 1.f;
    bool abort_commands = false;

    void to_xml(XmlWriter &writer) const;
};
} // namespace rbt
