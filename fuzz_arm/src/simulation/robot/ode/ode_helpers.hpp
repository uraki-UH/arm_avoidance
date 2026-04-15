#pragma once

#include <drawstuff/drawstuff.h>
#include <ode/ode.h>

#include <array>

// 正しい描画関数を選択   dReal は　double
// だが　描画関数は通常ではfloatなので対応させる
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

// Helper for alpha support if the DrawStuff version is old or customized
// Often DrawStuff doesn't have dsSetColorAlpha, only dsSetColor.
#ifndef dsSetColorAlpha
#define dsSetColorAlpha(r, g, b, a) dsSetColor(r, g, b)
#endif

namespace simulation {
static const char *DRAWSTUFF_TEXTURE_PATH = "../drawstuff/textures";
}

typedef std::array<dReal, 4> dReal4;
static const dReal4 dReal4_zero{};

// 描画関数　色指定　テクスチャ
[[maybe_unused]] static void drawGeom(dGeomID g, dReal4 color,
                                      bool wood_texture = false) {
  if (!g)
    return;

  // geomの形状タイプを取得
  int type = dGeomGetClass(g);
  if (type == dPlaneClass)
    return; // Planes are not placeable and cannot be drawn this way

  const dReal *pos = dGeomGetPosition(g);
  const dReal *R = dGeomGetRotation(g);

  // 色指定
  dsSetColorAlpha(color[0], color[1], color[2], color[3]);
  // テクスチャ
  if (wood_texture)
    dsSetTexture(DS_WOOD);

  // 形状タイプごとに描画
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths(g, sides);
    dsDrawBox(pos, R, sides);
  } else if (type == dSphereClass) {
    dsDrawSphere(pos, R, dGeomSphereGetRadius(g));
  } else if (type == dCapsuleClass) {
    dReal radius, length;
    dGeomCapsuleGetParams(g, &radius, &length);
    dsDrawCapsule(pos, R, length, radius);
  } else if (type == dCylinderClass) {
    dReal radius, length;
    dGeomCylinderGetParams(g, &radius, &length);
    dsDrawCylinder(pos, R, length, radius);
  }
}

// 描画関数　　デフォルト色　テクスチャ
[[maybe_unused]] static void drawGeom(dGeomID g) {
  if (!g)
    return;

  // geomの形状タイプを取得
  int type = dGeomGetClass(g);
  if (type == dPlaneClass)
    return;

  const dReal *pos = dGeomGetPosition(g);
  const dReal *R = dGeomGetRotation(g);

  // 形状タイプごとに描画
  if (type == dBoxClass) {
    dVector3 sides;
    dGeomBoxGetLengths(g, sides);
    dsDrawBox(pos, R, sides);
  } else if (type == dSphereClass) {
    dsDrawSphere(pos, R, dGeomSphereGetRadius(g));
  } else if (type == dCapsuleClass) {
    dReal radius, length;
    dGeomCapsuleGetParams(g, &radius, &length);
    dsDrawCapsule(pos, R, length, radius);
  } else if (type == dCylinderClass) {
    dReal radius, length;
    dGeomCylinderGetParams(g, &radius, &length);
    dsDrawCylinder(pos, R, length, radius);
  }
}
