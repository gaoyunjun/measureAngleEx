#include "measureAngleEx.h"
#include <osgDB/convertUTF>
#include <osgEarth/Map>

const double EQUATORLENGTH = 20037508.3427892;

MeasureAangleEx::MeasureAangleEx()
	:m_pLineFeatureNode(NULL)
	,m_pLabelNode(NULL)
	,m_pLineBufferFeatureNode(NULL)
{
	m_pRoot = new osg::Group;
	init();
}

MeasureAangleEx::~MeasureAangleEx()
{

}
void MeasureAangleEx::setMapNode( osgEarth::MapNode* mapNode )
{
	osgEarth::MapNode*pOldMapNode = getMapNode();
	if (pOldMapNode != mapNode)
	{
		if (pOldMapNode)
		{
			pOldMapNode->removeChild(m_pRoot.get());
		}
		m_pMapNode = mapNode;
		if (mapNode)
		{
			mapNode->addChild(m_pRoot.get());
		}
		rebuild();
	}
}


void MeasureAangleEx::mapMouseClick( const osg::Vec3d& point,bool bSecondPoint  )
{
	updatePoint(point,bSecondPoint);
}

void MeasureAangleEx::mapMouseMove( const osg::Vec3d& point,bool bSecondPoint )
{
	updatePoint(point,bSecondPoint);
}

void MeasureAangleEx::updateDisplay()
{
	if (m_pLineFeature->getGeometry()->size() ==3)
	{
		osg::Vec3d point = m_pLineFeature->getGeometry()->at(2);

		double angle = getAngle();

		m_pLabelNode->setPosition(osgEarth::GeoPoint(m_pMapNode->getMapSRS(), point[0], point[1]));
		std::stringstream ss;
		ss << "角度: "  << std::setw(8) << angle << "度" << std::endl;
		std::string str;
		str = ss.str();
		std::string name = osgDB::convertStringFromCurrentCodePageToUTF8(str);
		m_pLabelNode->setText(name);
	}
}

void MeasureAangleEx::clearFeature()
{
	if (m_pLineFeature)
	{
		m_pLineFeature->getGeometry()->clear();
		m_pLineFeatureNode->init();
		m_pLabelNode->setText("");
		m_vecPoint.clear();
		//Test
		m_pLineBufferFeatureNode->getFeature()->getGeometry()->clear();
		m_pLineBufferFeatureNode->init();
	}
}

void MeasureAangleEx::rebuild()
{
	if (!m_pMapNode.valid())
	{
		return;
	}

	if (m_pMapNode->getMapSRS()->isProjected())
	{
		m_lineStyle.getOrCreate<osgEarth::Symbology::LineSymbol>()
			->tessellationSize() = osgEarth::Distance(5.0,osgEarth::Units::NAUTICAL_MILES);
	}

	m_pLineFeature = new osgEarth::Features::Feature(
		new osgEarth::Annotation::LineString,m_pMapNode->getMapSRS(), m_lineStyle);
	m_pLineFeatureNode = new osgEarth::Annotation::FeatureNode(
		m_pMapNode.get(),m_pLineFeature);
	m_pLineFeatureNode->setMapNode(m_pMapNode.get());
	m_pRoot->addChild(m_pLineFeatureNode.get());

	m_pLabelNode = new osgEarth::Annotation::PlaceNode(
		m_pMapNode.get(), osgEarth::GeoPoint::GeoPoint(), "", m_textStyle);

	m_pLabelNode->setDynamic(true);
	m_pRoot->addChild(m_pLabelNode.get());

	//Test
	m_pLineBufferFeatureNode =  new osgEarth::Annotation::FeatureNode(
		m_pMapNode.get(),m_pLineFeature);
	m_pLineBufferFeatureNode->setMapNode(m_pMapNode.get());
	m_pRoot->addChild(m_pLineBufferFeatureNode.get());


	m_pRoot->getOrCreateStateSet()->setMode(GL_DEPTH_CLAMP,osg::StateAttribute::ON|osg::StateAttribute::OVERRIDE|osg::StateAttribute::PROTECTED);
	m_pRoot->getOrCreateStateSet()->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF|osg::StateAttribute::OVERRIDE|osg::StateAttribute::PROTECTED);
	m_pRoot->getOrCreateStateSet()->setRenderBinDetails(10000, "RenderBin");//设置渲染优先级

	
}

void MeasureAangleEx::init()
{
	//init style
	m_lineStyle.getOrCreate<osgEarth::Symbology::LineSymbol>()
		->stroke()->color() = osgEarth::Color::Yellow;
	m_lineStyle.getOrCreate<osgEarth::Symbology::LineSymbol>()
		->stroke()->width() = 2.0;
	m_lineStyle.getOrCreate<osgEarth::Symbology::LineSymbol>()
		->tessellation() = 150;
	m_lineStyle.getOrCreate<osgEarth::Symbology::AltitudeSymbol>()
		->clamping() = osgEarth::Symbology::AltitudeSymbol::CLAMP_TO_TERRAIN;
	m_lineStyle.getOrCreate<osgEarth::Symbology::AltitudeSymbol>()
		->technique() = osgEarth::Symbology::AltitudeSymbol::TECHNIQUE_GPU;
	m_lineStyle.getOrCreate<osgEarth::Symbology::AltitudeSymbol>()
		->verticalOffset() = 0.1;

	m_textStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->alignment()
		= osgEarth::Symbology::TextSymbol::ALIGN_LEFT_TOP;
	m_textStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->encoding()
		= osgEarth::Symbology::TextSymbol::ENCODING_UTF8;
	m_textStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->declutter()
		= true;
	m_textStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->haloOffset() = 0.05;
	m_textStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->fill()->color()
		= osg::Vec4(1.0, 0.0, 0.0, 0.8);
	m_textStyle.getOrCreate<osgEarth::Symbology::RenderSymbol>()->lighting() = false;
	m_textStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->font() = "fonts/simsun.ttc";
	m_textStyle.getOrCreate<osgEarth::Symbology::TextSymbol>()->size() = 25.0;

}

double MeasureAangleEx::getAngle()
{
	
	osg::Vec3d p0 = m_pLineFeature->getGeometry()->at(0);
	osg::Vec3d p1 = m_pLineFeature->getGeometry()->at(1);
	osg::Vec3d p2 = m_pLineFeature->getGeometry()->at(2);

    osg::Vec2d p00; 
	osg::Vec2d p11; 
	osg::Vec2d p22; 
	if (m_pMapNode->getMapSRS()->isProjected())
	{
		p00 = osg::Vec2d(p0[0],p0[1]);
		p11 = osg::Vec2d(p1[0],p1[1]);
		p22 = osg::Vec2d(p2[0],p2[1]);
	}
	else
	{
		p00 = lonLat2Mercator(osg::Vec2d(p0[0],p0[1]));
        p11 = lonLat2Mercator(osg::Vec2d(p1[0],p1[1]));
		p22 = lonLat2Mercator(osg::Vec2d(p2[0],p2[1])); 
	}
	double azimuthOne = bearing(p11,p00);
	double azimuthTwo = bearing(p11,p22);
	
	double angle = azimuthTwo -azimuthOne;

	
	if (std::fabs(angle)> osg::PI)
	{
		if (angle < 0)
		{
			angle = osg::PI +(angle + osg::PI);
		}
		else
		{
			angle = -osg::PI +(angle - osg::PI);
		}
	}
	return  osg::RadiansToDegrees(angle);
}

void MeasureAangleEx::initFeatureNode()
{
	m_pLineFeatureNode->init();
	osgEarth::Symbology::Geometry*line= m_pLineFeature->getGeometry();
	if (line)
	{
		osg::ref_ptr<osgEarth::Symbology::Geometry>pBufer =NULL;
		bool buffer = line->buffer(1000,pBufer);
		if (buffer)
		{
			osgEarth::Features::Feature* ferure=new osgEarth::Features::Feature(pBufer,m_pMapNode->getMapSRS(), m_lineStyle);
			m_pLineBufferFeatureNode->setFeature(ferure);
			m_pLineBufferFeatureNode->init();
		}

	}
}

void MeasureAangleEx::updatePoint(const osg::Vec3d &point,bool bSecondPoint)
{
	if (bSecondPoint)
	{
		if (m_pLineFeature->getGeometry()->size() == 1)
		{
			m_pLineFeature->getGeometry()->push_back(point);
		}
		else
		{
			m_pLineFeature->getGeometry()->back() = point;
		}

	}
	else
	{

		if (m_pLineFeature->getGeometry()->size() ==3)
		{
			m_pLineFeature->getGeometry()->back() = point;
		}
		else
		{
			m_pLineFeature->getGeometry()->push_back(point);
		}
	}
	m_pLineFeatureNode->init();

	osgEarth::Symbology::Geometry*line= m_pLineFeature->getGeometry();
	if (line)
	{
		osg::ref_ptr<osgEarth::Symbology::Geometry>pBufer =NULL;
		bool buffer = line->buffer(2000,pBufer);
		if (buffer)
		{
			osgEarth::Features::Feature* ferure=new osgEarth::Features::Feature(pBufer,m_pMapNode->getMapSRS(), m_lineStyle);
			m_pLineBufferFeatureNode->setFeature(ferure);
			m_pLineBufferFeatureNode->init();
		}

	}


}

osg::Vec2d MeasureAangleEx::lonLat2Mercator( osg::Vec2d lonlat )
{
	osg::Vec2d point;
	double x = lonlat.x()*EQUATORLENGTH/180.0; 
	double y = log(tan((90 + lonlat.y())*osg::PI/360.0))/(osg::PI/180.0);
	y = y*EQUATORLENGTH/180.0;
	point.x() = x;
	point.y() = y;
	return point;
}

int MeasureAangleEx::getPointCount()
{
	return m_pLineFeature->getGeometry()->size();
}

double MeasureAangleEx::bearing( osg::Vec2d p1,osg::Vec2d p2 )
{
	double bearing;
	double dx = p2.x() - p1.x();
	double dy = p2.y() - p1.y();
	bearing = atan2(dx,dy);
	return bearing;
}

MeasureAangleExHandler::MeasureAangleExHandler( osgEarth::MapNode* mapNode)
	:m_pMapNode(mapNode)
	,m_intersectionMask(0xffffffff )
	,m_bMouseDown(false)
	,m_bGotFirstLocation(false)
	,m_bLastPointTemporary(false)
	,m_bFinished(false)
	,m_bGotSecondLocation(false)
	,m_bEnable(false)
	,m_iMouseButton(osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
{
	m_pMeasureAngle = new MeasureAangleEx();
	m_pMeasureAngle->setMapNode(m_pMapNode.get());
}

MeasureAangleExHandler::~MeasureAangleExHandler()
{
	m_pMeasureAngle->setMapNode(0L);
}

bool MeasureAangleExHandler::handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa )
{
	if (ea.getHandled()||!m_bEnable )
	{
		return false;
	}
	osgViewer::View*view = static_cast<osgViewer::View*>(aa.asView());
	if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == m_iMouseButton)
	{
		m_bMouseDown = true;

		m_fMouseDownX = ea.getX();
		m_fMouseDownY =ea.getY();
	}
	else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE && ea.getButton() == m_iMouseButton)
	{
		float eps =1.0f;
		m_bMouseDown =false;
		if (osg::equivalent(ea.getX(), m_fMouseDownX, eps) && osg::equivalent(ea.getY(), m_fMouseDownY, eps))
		{
			osg::Vec3d lonlaH;
			osg::Vec3d worldXYZ;
			if (getLocationAt(view, ea.getX(), ea.getY(), lonlaH))
			{
				if (!m_bGotFirstLocation)
				{
					m_bFinished = false;
					m_pMeasureAngle->clearFeature();
					m_bGotFirstLocation = true;
					m_pMeasureAngle->mapMouseClick(lonlaH);
				}
				else
				{
					if (m_pMeasureAngle->getPointCount() == 2)
					{
						m_pMeasureAngle->mapMouseMove(lonlaH,true);
						m_bGotSecondLocation = true;
					}
					else
					{
						m_pMeasureAngle->mapMouseClick(lonlaH);
					}

					aa.requestRedraw();
					if (m_bFinished)
					{
						m_bGotFirstLocation = false;
						m_bGotSecondLocation = false;
					}

					if (m_pMeasureAngle->getPointCount() ==3)
					{
						m_bFinished = true;
						m_bGotFirstLocation = false;
						m_bGotSecondLocation = false;
					}
				}
			}

		}
	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE && ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
	{
		m_pMeasureAngle->clearFeature();
		m_bFinished = true;
		m_bGotFirstLocation = false;
		m_bGotSecondLocation = false;

	}

	else if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK && ea.getButton() == m_iMouseButton)
	{
		if (m_pMeasureAngle->getPointCount() ==3)
		{
			m_bFinished = true;
			aa.requestRedraw();
			return true;
		}
	}
	else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
	{

		if (m_bGotFirstLocation &&!m_bFinished)
		{
			osg::Vec3d lonlaH;
			osg::Vec3d worldXYZ;
			if (getLocationAt(view, ea.getX(), ea.getY(), lonlaH))
			{
				if (!m_bGotSecondLocation)
				{
					m_pMeasureAngle->mapMouseMove(lonlaH,true);
				}
				else
				{
					m_pMeasureAngle->mapMouseMove(lonlaH);
					m_pMeasureAngle->updateDisplay();
				}
				aa.requestRedraw();
			}
		}
	}
	return false;
}



bool MeasureAangleExHandler::getLocationAt( osgViewer::View* view, double x, double y, osg::Vec3d &lonlaH)
{
	osg::Vec3d world;
	if (m_pMapNode->getTerrain()->getWorldCoordsUnderMouse(view, x, y, world))
	{
		osgEarth::GeoPoint mapPoint;
		mapPoint.fromWorld( m_pMapNode->getMapSRS(), world );
		lonlaH[0] = mapPoint.x();
		lonlaH[1] = mapPoint.y();
		return true;
    }
	return false;
}

void MeasureAangleExHandler::setEnable( bool bEnable )
{
	m_bEnable = bEnable;
}

bool MeasureAangleExHandler::getEnable()
{
	return m_bEnable;
}
