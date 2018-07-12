#include "raster_window.hpp"

RasterWindow::RasterWindow(QWindow *parent)
  : QWindow(parent), update_pending_(false)
{
  this->backing_store_ = new QBackingStore(this);

  this->timer_.start(10, this);

  this->create();
  this->setGeometry(100, 100, 640, 640);
  
  this->boid_poly_ << QPointF(0, 0);
  this->boid_poly_ << QPointF(-5, -5);
  this->boid_poly_ << QPointF(10, 0);
  this->boid_poly_ << QPointF(-5, 5);
  this->boid_poly_ << QPointF(0, 0);
}



void RasterWindow::render(QPainter* painter)
{
  auto& swarm = SwarmApplication::instance()->swarm_;
  auto& world = SwarmApplication::instance()->world_;

  double xscale = double(this->width()) / world.get_dimensions()[0];
  double yscale = double(this->height()) / world.get_dimensions()[1];

  painter->translate(this->width() / 2, this->height() / 2);
  painter->scale(xscale, -yscale);
  QBrush brush(Qt::black);
  painter->setPen(QPen(brush, 0.5));

  boids::Boid::Coord x_axis;
  x_axis << 1.0, 0.0, 0.0;

  for (boids::Boid& boid : swarm) {
    QTransform rotate;
    double phi = acos(x_axis.dot(boid.get_forward()));
    auto axis = boid.get_forward().cross(x_axis);
    phi = (axis[2] < 0) ? phi : -phi;
    rotate.rotateRadians(phi);
    QTransform translate;
    translate.translate(boid.get_x()[0], boid.get_x()[1]);
    QPolygonF boid_poly = rotate.map(this->boid_poly_);
    boid_poly = translate.map(boid_poly);
    QPainterPath path;
    path.addPolygon(boid_poly);
    painter->setPen(QColor(Qt::black));
    painter->drawPolygon(boid_poly);
    painter->fillPath(path, brush);
    
    painter->setPen(QPen(QBrush(Qt::blue), 0.5));
    this->drawViewRange(painter, boid, boid.view_angle(), boid.sight_range());
    painter->setPen(QPen(QBrush(Qt::red), 0.5));
    this->drawViewRange(painter, boid, boid.view_angle(), boid.min_dist());
  }
}



void RasterWindow::renderLater()
{
  if (!this->update_pending_) {
    this->update_pending_ = true;
    QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
  }
}



void RasterWindow::renderNow()
{
  if (!this->isExposed())
    return;
  
  QRect rect(0, 0, this->width(), this->height());
  this->backing_store_->beginPaint(rect);

  QPaintDevice* device = this->backing_store_->paintDevice();
  QPainter painter(device);
  
  painter.fillRect(0, 0, this->width(), this->height(), Qt::white);
  this->render(&painter);

  this->backing_store_->endPaint();
  this->backing_store_->flush(rect);
}



void RasterWindow::updateSwarm()
{
  for (boids::Boid& boid : SwarmApplication::instance()->swarm_)
    boid.step_setup(SwarmApplication::instance()->swarm_);
  for (boids::Boid& boid : SwarmApplication::instance()->swarm_)
    boid.step(0.01);
}



bool RasterWindow::event(QEvent* event)
{
  if (event->type() == QEvent::UpdateRequest) {
    this->update_pending_ = false;
    this->renderNow();
    return true;
  }
  return QWindow::event(event);
}



void RasterWindow::resizeEvent(QResizeEvent* event)
{
  this->backing_store_->resize(event->size());
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
  if (event->timerId() == this->timer_.timerId()) {
    this->updateSwarm();
    this->renderNow();
  }
  else
    QWindow::timerEvent(event);
}



void RasterWindow::drawViewRange(QPainter* painter, const boids::Boid& boid,
				 const double angle, const double range)
{  
  QRectF rect(boid.get_x()[0] - range, boid.get_x()[1] - range, 2 * range, 2 * range);
  boids::Boid::Coord x_axis;
  x_axis << 1, 0, 0;
  double phi = acos(x_axis.dot(boid.get_forward()));
  auto axis = boid.get_forward().cross(x_axis);
  phi = (axis[2] < 0) ? phi : -phi;
  painter->drawPie(rect,
		   - 16 * 180.0 / boids::math::pi
		   * (angle + phi),
		   16 * 180.0 / boids::math::pi * 2 * angle);
}
