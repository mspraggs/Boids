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

  void updateSwarm();

protected:
  bool event(QEvent* event);

  void resizeEvent(QResizeEvent* event);
  void exposeEvent(QExposeEvent* event);
  void timerEvent(QTimerEvent* event);

private:
  void drawSwarm();

  QBackingStore* _backing_store;
  bool _update_pending;

  QBasicTimer _timer;
  QPolygonF _boid_poly;
};

#endif
