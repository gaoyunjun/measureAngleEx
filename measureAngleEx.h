/***************************************************************************** 
    *  @COPYRIGHT NOTICE 
    *  @Copyright (c) 2020, GYJ
    *  @All rights reserved 
 
    *  @file     : MeasureAangleEx.h
    *  @version  : ver 1.0 
    
    *  @author   : GYJ 
    *  @date     : 2020/06/13  11:27
    *  @brief    : 
*****************************************************************************/ 
#ifndef MEASUREANGLEEX_H
#define MEASUREANGLEEX_H
#include <osgViewer/View>

#include <osgEarth/GeoMath>
#include <osgEarth/GeoData>

#include <osgEarthAnnotation/EllipseNode>
#include <osgEarthAnnotation/FeatureNode>
#include <osgEarthAnnotation/PlaceNode>
#include <osgEarthSymbology/Geometry>
#include <osgEarth/MapNode>
#include <osgGA/GUIEventHandler>

using namespace osgEarth::Symbology;

class MeasureAangleEx: public osg::Referenced
{
public:
	MeasureAangleEx();
	~MeasureAangleEx();

	void mapMouseClick(const osg::Vec3d&  point,bool bSecondPoint = false);

	void mapMouseMove(const osg::Vec3d& point,bool bSecondPoint = false);

	void initFeatureNode();

	void clearFeature();

	void setMapNode( osgEarth::MapNode* mapNode );

	osgEarth::MapNode* getMapNode() { return m_pMapNode.get(); }
	void updateDisplay();

	int getPointCount();

	

protected:
	void rebuild();
	void init();
	double getAngle();

	void updatePoint(const osg::Vec3d &point,bool bSecondPoint = false);

	osg::Vec2d lonLat2Mercator(osg::Vec2d lonlat);

	double bearing(osg::Vec2d p1,osg::Vec2d p2);

private:
	osg::Vec4f _pathColor;
	osg::ref_ptr<osgEarth::Symbology::LineString>			    m_pLine;
	osgEarth::Symbology::Style                                  m_lineStyle;
	osgEarth::Symbology::Style                                  m_textStyle;
	osg::observer_ptr<osg::Group>								m_pRoot;
	osg::observer_ptr<osgEarth::MapNode>						m_pMapNode;
	osg::ref_ptr<osgEarth::Features::Feature>					m_pLineFeature;
	osg::ref_ptr<osgEarth::Annotation::FeatureNode>				m_pLineFeatureNode;
	osg::ref_ptr<osgEarth::Annotation::FeatureNode>				m_pLineBufferFeatureNode;
	osg::ref_ptr<osgEarth::Annotation::PlaceNode>		        m_pLabelNode;
	std::vector<osg::Vec3d>                                     m_vecPoint;
};
class MeasureAangleExHandler : public osgGA::GUIEventHandler
{
public:
	MeasureAangleExHandler(osgEarth::MapNode* mapNode);

	~MeasureAangleExHandler();

	bool handle( const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

	void setIntersectionMask( osg::Node::NodeMask intersectionMask ) { m_intersectionMask = intersectionMask; }
	osg::Node::NodeMask getIntersectionMask() const { return m_intersectionMask;}

	void setEnable(bool bEnable);

	bool getEnable();

protected:
	bool getLocationAt(osgViewer::View* view, double x, double y, osg::Vec3d &lonlaH);
private:
	osg::ref_ptr<MeasureAangleEx>						    m_pMeasureAngle;
	osg::ref_ptr<osgEarth::MapNode>				        m_pMapNode;
	int                                                 m_iMouseButton;
	osg::Vec3d											m_vLastPoint;
	osg::Vec3d											m_vCurrentPoint;
	osg::Node::NodeMask                                 m_intersectionMask;
	bool                                                m_bLastPointTemporary;
	bool                                                m_bGotFirstLocation;
	bool                                                m_bGotSecondLocation;
	bool                                                m_bFinished;
	bool                                                m_bMouseDown;
	bool                                                m_bEnable;
	float                                               m_fMouseDownX;
	float                                               m_fMouseDownY;
};

#endif //MEASUREANGLEEX_H