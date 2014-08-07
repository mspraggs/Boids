#ifndef RASTER_WINDOW_HPP
#define RASTER_WINDOW_HPP

#include <QtGui>

#include <swarm_application.hpp>

class RasterWindow : public QWindow
{

public:
  explicit RasterWindow(QWindow* parent = 0);
  
  virtual void render(QPainter* painter);

public slots:
  void renderLater();
  void renderNow();

protected:
  bool event(QEvent* event);

  void resizeEvent(QResizeEvent* event);
  void exposeEvent(QExposeEvent* event);

private:
  QBackingStore* _backing_store;
  bool _update_pending;
};

#endif
