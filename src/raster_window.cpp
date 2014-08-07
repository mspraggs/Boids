#include <raster_window.hpp>

RasterWindow::RasterWindow(QWindow *parent)
  : QWindow(parent), _update_pending(false)
{
  this->_backing_store = new QBackingStore(this);

  this->_timer.start(10, this);

  this->create();
  this->setGeometry(100, 100, 480, 480);
}



void RasterWindow::render(QPainter* painter)
{
  painter->drawText(QRectF(0, 0, this->width(), this->height()),
		    Qt::AlignCenter, QStringLiteral("QWindow"));
}



void RasterWindow::renderLater()
{
  if (!this->_update_pending) {
    this->_update_pending = true;
    QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
  }
}



void RasterWindow::renderNow()
{
  if (!this->isExposed())
    return;
  
  QRect rect(0, 0, this->width(), this->height());
  this->_backing_store->beginPaint(rect);

  QPaintDevice* device = this->_backing_store->paintDevice();
  QPainter painter(device);
  
  painter.fillRect(0, 0, this->width(), this->height(), Qt::white);
  this->render(&painter);

  this->_backing_store->endPaint();
  this->_backing_store->flush(rect);
}



void RasterWindow::updateSwarm()
{
  for (boids::Boid& boid : SwarmApplication::instance()->swarm)
    boid.step_setup(SwarmApplication::instance()->swarm);
  for (boids::Boid& boid : SwarmApplication::instance()->swarm)
    boid.step(0.01);
}



bool RasterWindow::event(QEvent* event)
{
  if (event->type() == QEvent::UpdateRequest) {
    this->_update_pending = false;
    this->renderNow();
    return true;
  }
  return QWindow::event(event);
}



void RasterWindow::resizeEvent(QResizeEvent* event)
{
  this->_backing_store->resize(event->size());
  if (this->isExposed())
    this->renderNow();
}



void RasterWindow::exposeEvent(QExposeEvent* event)
{
  if (this->isExposed())
    this->renderNow();
}



void RasterWindow::timerEvent(QTimerEvent* event)
{
  if (event->timerId() == this->_timer.timerId())
    this->updateSwarm();
  else
    QWindow::timerEvent(event);
}
