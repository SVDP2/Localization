// Copyright 2024 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ESKF_LOCALIZATION__RVIZ_OVERLAY__OVERLAY_UTILS_HPP_
#define ESKF_LOCALIZATION__RVIZ_OVERLAY__OVERLAY_UTILS_HPP_

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

namespace eskf_localization
{
namespace rviz_overlay
{

class OverlayObject;

class ScopedPixelBuffer
{
public:
  explicit ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer);
  virtual ~ScopedPixelBuffer();
  virtual Ogre::HardwarePixelBufferSharedPtr getPixelBuffer();
  virtual QImage getQImage(unsigned int width, unsigned int height);
  virtual QImage getQImage(OverlayObject & overlay);
  virtual QImage getQImage(unsigned int width, unsigned int height, QColor & bg_color);
  virtual QImage getQImage(OverlayObject & overlay, QColor & bg_color);

protected:
  Ogre::HardwarePixelBufferSharedPtr pixel_buffer_;
};

enum class VerticalAlignment : uint8_t { CENTER, TOP, BOTTOM };

enum class HorizontalAlignment : uint8_t { LEFT, RIGHT, CENTER };

/**
 * Helper class for realizing an overlay object on top of the rviz 3D panel.
 */
class OverlayObject
{
public:
  using SharedPtr = std::shared_ptr<OverlayObject>;

  explicit OverlayObject(const std::string & name);
  virtual ~OverlayObject();

  virtual std::string getName() const;
  virtual void hide();
  virtual void show();
  virtual bool isTextureReady() const;
  virtual void updateTextureSize(unsigned int width, unsigned int height);
  virtual ScopedPixelBuffer getBuffer();
  virtual void setPosition(
    double hor_dist, double ver_dist, HorizontalAlignment hor_alignment = HorizontalAlignment::LEFT,
    VerticalAlignment ver_alignment = VerticalAlignment::TOP);
  virtual void setDimensions(double width, double height);
  virtual bool isVisible() const;
  virtual unsigned int getTextureWidth() const;
  virtual unsigned int getTextureHeight() const;

protected:
  const std::string name_;
  Ogre::Overlay * overlay_;
  Ogre::PanelOverlayElement * panel_;
  Ogre::MaterialPtr panel_material_;
  Ogre::TexturePtr texture_;
};

}  // namespace rviz_overlay
}  // namespace eskf_localization

#endif  // ESKF_LOCALIZATION__RVIZ_OVERLAY__OVERLAY_UTILS_HPP_
