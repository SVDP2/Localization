#include "relative_localization_eskf/rviz_overlay/overlay_utils.hpp"

#include <rviz_common/logging.hpp>

#include <cstring>

namespace relative_localization_eskf
{
namespace rviz_overlay
{

ScopedPixelBuffer::ScopedPixelBuffer(Ogre::HardwarePixelBufferSharedPtr pixel_buffer)
: pixel_buffer_(pixel_buffer)
{
  if (pixel_buffer_) {
    pixel_buffer_->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  }
}

ScopedPixelBuffer::~ScopedPixelBuffer()
{
  if (pixel_buffer_) {
    pixel_buffer_->unlock();
  }
}

Ogre::HardwarePixelBufferSharedPtr ScopedPixelBuffer::getPixelBuffer()
{
  return pixel_buffer_;
}

QImage ScopedPixelBuffer::getQImage(unsigned int width, unsigned int height)
{
  const Ogre::PixelBox & pixel_box = pixel_buffer_->getCurrentLock();
  auto * data = static_cast<Ogre::uint8 *>(pixel_box.data);
  std::memset(data, 0, width * height * 4);
  return QImage(data, width, height, QImage::Format_ARGB32);
}

QImage ScopedPixelBuffer::getQImage(OverlayObject & overlay)
{
  return getQImage(overlay.getTextureWidth(), overlay.getTextureHeight());
}

OverlayObject::OverlayObject(const std::string & name)
: name_(name)
{
  Ogre::OverlayManager * overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  overlay_ = overlay_mgr->create(name_);
  panel_ = static_cast<Ogre::PanelOverlayElement *>(
    overlay_mgr->createOverlayElement("Panel", name_ + "Panel"));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);

  panel_material_ = Ogre::MaterialManager::getSingleton().create(
    name_ + "Material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  panel_->setMaterialName(panel_material_->getName());
  overlay_->add2D(panel_);
}

OverlayObject::~OverlayObject()
{
  Ogre::OverlayManager * overlay_mgr = Ogre::OverlayManager::getSingletonPtr();
  if (overlay_mgr) {
    overlay_mgr->destroyOverlayElement(panel_);
    overlay_mgr->destroy(overlay_);
  }
  if (panel_material_) {
    panel_material_->unload();
    Ogre::MaterialManager::getSingleton().remove(panel_material_->getName());
  }
}

void OverlayObject::hide()
{
  if (overlay_->isVisible()) {
    overlay_->hide();
  }
}

void OverlayObject::show()
{
  if (!overlay_->isVisible()) {
    overlay_->show();
  }
}

bool OverlayObject::isTextureReady() const
{
  return texture_ != nullptr;
}

void OverlayObject::updateTextureSize(unsigned int width, unsigned int height)
{
  if (width == 0) {
    RVIZ_COMMON_LOG_WARNING_STREAM("overlay width was 0; forcing 1");
    width = 1;
  }
  if (height == 0) {
    RVIZ_COMMON_LOG_WARNING_STREAM("overlay height was 0; forcing 1");
    height = 1;
  }

  const std::string texture_name = name_ + "Texture";
  if (!isTextureReady() || width != texture_->getWidth() || height != texture_->getHeight()) {
    if (isTextureReady()) {
      Ogre::TextureManager::getSingleton().remove(texture_name);
      panel_material_->getTechnique(0)->getPass(0)->removeAllTextureUnitStates();
    }
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      texture_name,
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D,
      width,
      height,
      0,
      Ogre::PF_A8R8G8B8,
      Ogre::TU_DEFAULT);
    panel_material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_name);
    panel_material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  }
}

ScopedPixelBuffer OverlayObject::getBuffer()
{
  return ScopedPixelBuffer(isTextureReady() ? texture_->getBuffer() : Ogre::HardwarePixelBufferSharedPtr());
}

void OverlayObject::setPosition(
  double hor_dist, double ver_dist,
  HorizontalAlignment hor_alignment,
  VerticalAlignment ver_alignment)
{
  double left = 0.0;
  double top = 0.0;
  switch (hor_alignment) {
    case HorizontalAlignment::LEFT:
      panel_->setHorizontalAlignment(Ogre::GHA_LEFT);
      left = hor_dist;
      break;
    case HorizontalAlignment::CENTER:
      panel_->setHorizontalAlignment(Ogre::GHA_CENTER);
      left = hor_dist - panel_->getWidth() / 2.0;
      break;
    case HorizontalAlignment::RIGHT:
      panel_->setHorizontalAlignment(Ogre::GHA_RIGHT);
      left = -hor_dist - panel_->getWidth();
      break;
  }
  switch (ver_alignment) {
    case VerticalAlignment::TOP:
      panel_->setVerticalAlignment(Ogre::GVA_TOP);
      top = ver_dist;
      break;
    case VerticalAlignment::CENTER:
      panel_->setVerticalAlignment(Ogre::GVA_CENTER);
      top = ver_dist - panel_->getHeight() / 2.0;
      break;
    case VerticalAlignment::BOTTOM:
      panel_->setVerticalAlignment(Ogre::GVA_BOTTOM);
      top = -ver_dist - panel_->getHeight();
      break;
  }
  panel_->setPosition(left, top);
}

void OverlayObject::setDimensions(double width, double height)
{
  panel_->setDimensions(width, height);
}

bool OverlayObject::isVisible() const
{
  return overlay_->isVisible();
}

unsigned int OverlayObject::getTextureWidth() const
{
  return isTextureReady() ? texture_->getWidth() : 0;
}

unsigned int OverlayObject::getTextureHeight() const
{
  return isTextureReady() ? texture_->getHeight() : 0;
}

}  // namespace rviz_overlay
}  // namespace relative_localization_eskf
