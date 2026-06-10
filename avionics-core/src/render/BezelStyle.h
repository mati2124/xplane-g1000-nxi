#pragma once

#include "avionics/Color.h"
#include "avionics/Renderer.h"

// Shared look of the physical bezel controls drawn by the standalone shell
// around the screen (the right key column and the bottom softkey row), modeled
// on the GDU 104X's molded charcoal keys (G1000 Pilot's Guide for the Diamond
// DA40, Figure 1-1 / "Softkey Selection Keys").
namespace avionics::bezel {

// Dark metallic face of the bezel itself (the frame the keys sit in).
inline constexpr Color kFaceTop{0.13f, 0.14f, 0.16f, 1.0f};
inline constexpr Color kFaceBottom{0.06f, 0.065f, 0.08f, 1.0f};

// Draws one physical key cap: a rounded charcoal button with a soft top sheen
// and a dark outline. press (0..1) lightens the cap as click feedback.
void drawKeyFace(Renderer& r, float x, float y, float w, float h, float press);

}  // namespace avionics::bezel
