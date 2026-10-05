#pragma once

namespace owlet::hal
{

/// Called from an interrupt when an input becomes active. Whatever
/// implements this must do very little: no waiting, no floating point,
/// nothing that is not safe against the code it interrupted.
class IEdgeListener
{
public:
    virtual ~IEdgeListener() = default;

    virtual void onEdge() = 0;
};

} // namespace owlet::hal
