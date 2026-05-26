#ifndef ARUCO_IMU_ESKF_LOCALIZATION_CPP__RVIZ_OVERLAY__OVERLAY_UTILS_HPP_
#define ARUCO_IMU_ESKF_LOCALIZATION_CPP__RVIZ_OVERLAY__OVERLAY_UTILS_HPP_

#include <QColor>
#include <QImage>

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include <memory>
#include <string>

namespace relative_localization_eskf
{
namespace rviz_overlay
{

class OverlayObject;

class ScopedPixelBuffer
{
public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  ~ScopedPixelBuffer();
  Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
  QImage getQImage(unsigned int width, unsigned int height);
  QImage getQImage(OverlayObject & overlay);

private:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
};

enum class VerticalAlignment : uint8_t {CENTER, TOP, BOTTOM};
enum class HorizontalAlignment : uint8_t {LEFT, RIGHT, CENTER};

class OverlayObject
{
public:
  explicit OverlayObject(const std::string & name);
  ~OverlayObject();

  void hide();
  void show();
  bool isTextureReady() const;
  void updateTextureSize(unsigned int width, unsigned int height);
  ScopedPixelBuffer getBuffer();
  void setPosition(
    double hor_dist, double ver_dist,
    HorizontalAlignment hor_alignment = HorizontalAlignment::LEFT,
    VerticalAlignment ver_alignment = VerticalAlignment::TOP);
  void setDimensions(double width, double height);
  bool isVisible() const;
  unsigned int getTextureWidth() const;
  unsigned int getTextureHeight() const;

private:
  const std::string name_;
  Ogre::Overlay * overlay_{nullptr};
  Ogre::PanelOverlayElement * panel_{nullptr};
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
};

}  // namespace rviz_overlay
}  // namespace relative_localization_eskf

#endif  // ARUCO_IMU_ESKF_LOCALIZATION_CPP__RVIZ_OVERLAY__OVERLAY_UTILS_HPP_
