#include <raster_window.hpp>

RasterWindow::RasterWindow(QWindow *parent)
  : QWindow(parent), _update_pending(false)
{
  this->_backing_store = new QBackingStore(this);

  this->_timer.start(10, this);

  this->create();
  this->setGeometry(100, 100, 640, 640);
  
  this->_boid_poly << QPointF(0, 0);
  this->_boid_poly << QPointF(-5, -5);
  this->_boid_poly << QPointF(10, 0);
  this->_boid_poly << QPointF(-5, 5);
  this->_boid_poly << QPointF(0, 0);
}



void RasterWindow::render(QPainter* painter)
{
  std::vector<boids::Boid>& swarm = SwarmApplication::instance()->swarm;

  double xscale = double(this->width()) / swarm[0].x_span();
  double yscale = double(this->height()) / swarm[0].y_span();

  painter->translate(this->width() / 2, this->height() / 2);
  painter->scale(xscale, -yscale);
  QBrush brush(Qt::SolidPattern);
  painter->setPen(QPen(brush, 0.5));

  for (boids::Boid& boid : swarm) {
    QTransform rotate;
    rotate.rotateRadians(boid.v_theta());
    QTransform translate;
    translate.translate(boid.r_x(), boid.r_y());
    QPolygonF boid_poly = rotate.map(this->_boid_poly);
    boid_poly = translate.map(boid_poly);
    QPainterPath path;
    path.addPolygon(boid_poly);
    painter->drawPolygon(boid_poly);
    painter->fillPath(path, brush);
    painter->drawEllipse(QPointF(boid.r_x(), boid.r_y()),
			 boid.sight_range(),
			 boid.sight_range());
    painter->drawEllipse(QPointF(boid.r_x(), boid.r_y()),
			 boid.min_dist(),
			 boid.min_dist());
  }
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
  if (event->timerId() == this->_timer.timerId()) {
    this->updateSwarm();
    this->renderNow();
  }
  else
    QWindow::timerEvent(event);
}
