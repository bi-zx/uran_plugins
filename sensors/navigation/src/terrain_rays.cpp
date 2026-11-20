// Map-frame measured rays for rectangular bases. The input and clock belong to the sensor.
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace {
using Point = std::array<double,3>;
using Tile = std::pair<int,int>;
using RayKey = std::array<int,7>;
using Stamp = long long;
using Mask = std::map<int,std::uint64_t>;
struct Ray { Point o, p; int kind; Stamp stamp; unsigned visit = 0; bool superseded = false; };
struct Column {
    double floor = 0.;
    Stamp floor_stamp = 0, barrier = 0, cleared_at = 0;
    Mask air;
    std::set<Ray*> hits;
    bool blocked = false, clear = false;
};
struct Config {
    double size, step, height, range, slope, dz;
    std::vector<std::pair<double,double>> bands;
};
struct Frame {
    Stamp stamp;
    std::map<Tile,Column> columns;
    std::vector<Ray> rays;
};
struct History {
    Config cfg;
    std::map<Tile,Column> columns;
    std::map<RayKey,Ray> rays;
    std::map<Tile,std::set<Ray*>> index;
    unsigned query = 0;
};

int tile(double v, double size) { return static_cast<int>(std::floor(v/size)); }

// Signed map height bins, in blocks of 64. Only bins crossed by a measured ray are set.
int word(int bin) { return bin >= 0 ? bin/64 : -1-((-1-bin)/64); }
std::uint64_t bits(int start, int stop) {
    return (~std::uint64_t(0) << start) & (~std::uint64_t(0) >> (63-stop));
}
void add_air(Mask& mask, int a, int b) {
    for (int w=word(a); a<=b && w<=word(b); ++w)
        mask[w] |= bits(std::max(0,a-w*64), std::min(63,b-w*64));
}
bool has_air(const Mask& mask, int a, int b) {
    for (int w=word(a); a<=b && w<=word(b); ++w) {
        auto it = mask.find(w);
        if (it!=mask.end() && (it->second & bits(std::max(0,a-w*64),std::min(63,b-w*64)))) return true;
    }
    return false;
}
bool clear_column(const Column& col, const Config& cfg) {
    if (!col.floor_stamp) return false;
    for (auto band:cfg.bands) {
        int a = static_cast<int>(std::ceil((col.floor+band.first)/cfg.dz-1e-9));
        int b = static_cast<int>(std::floor((col.floor+band.second)/cfg.dz+1e-9))-1;
        if (!has_air(col.air,a,b)) return false;
    }
    return true;
}

template<class Visit>
void traverse(const Ray& ray, double size, Visit visit) {
    const double dx=ray.p[0]-ray.o[0], dy=ray.p[1]-ray.o[1];
    int ix=tile(ray.o[0],size), iy=tile(ray.o[1],size);
    const double inf=std::numeric_limits<double>::infinity();
    double tx=dx ? ((ix+(dx>0))*size-ray.o[0])/dx : inf;
    double ty=dy ? ((iy+(dy>0))*size-ray.o[1])/dy : inf;
    const double dtx=dx ? size/std::abs(dx) : inf, dty=dy ? size/std::abs(dy) : inf;
    for (double t=0.; t<1.;) {
        double stop=std::min({tx,ty,1.});
        if (stop>t) visit(Tile(ix,iy),t,stop);
        if (stop>=1.) break;
        if (tx<=stop) { ix+=dx>0 ? 1 : -1; tx+=dtx; }
        if (ty<=stop) { iy+=dy>0 ? 1 : -1; ty+=dty; }
        t=stop;
    }
}
void project(Frame& frame, const Config& cfg, const Point& origin, Point p,
             const Point& ground, double bx, double by, bool returned) {
    double dx=p[0]-origin[0], dy=p[1]-origin[1], dz=p[2]-origin[2];
    double length=std::sqrt(dx*dx+dy*dy+dz*dz);
    if (!length) return;
    double fraction=std::min(1.,cfg.range/length);
    if (std::hypot(origin[0]+fraction*dx-bx,origin[1]+fraction*dy-by)>5.) {
        double aa=dx*dx+dy*dy, b=dx*(origin[0]-bx)+dy*(origin[1]-by);
        double cc=std::pow(origin[0]-bx,2)+std::pow(origin[1]-by,2)-25.;
        fraction=(-b+std::sqrt(b*b-aa*cc))/aa;
    }
    if (fraction<1.) {
        p={origin[0]+fraction*dx,origin[1]+fraction*dy,origin[2]+fraction*dz};
        returned=false;
    }
    Ray ray{origin,p,0,frame.stamp};
    Tile end(tile(p[0],cfg.size),tile(p[1],cfg.size));
    double relative=p[2]-ground[2];
    if (returned && std::abs(relative)<=cfg.step) {
        ray.kind=1;
        auto& col=frame.columns[end];
        col.floor=col.floor_stamp ? std::min(col.floor,p[2]) : p[2];
        col.floor_stamp=frame.stamp;
    } else if (returned && relative<=cfg.height+cfg.step) {
        ray.kind=-1;
        frame.columns[end].blocked=true;
    }
    int base=tile(ground[2]-cfg.step,cfg.dz);
    int top=static_cast<int>(std::ceil((ground[2]+cfg.height+cfg.step)/cfg.dz));
    double z0=origin[2]/cfg.dz, delta=(p[2]-origin[2])/cfg.dz;
    traverse(ray,cfg.size,[&](Tile xy,double t,double stop) {
        double za=z0+t*delta, zb=z0+stop*delta;
        int lo=std::max(base,static_cast<int>(std::floor(std::min(za,zb))));
        int hi=std::min(top,delta ? static_cast<int>(std::ceil(std::max(za,zb)))-1 : static_cast<int>(std::floor(za)));
        if (lo<=hi) add_air(frame.columns[xy].air,lo,hi);
    });
    frame.rays.push_back(ray);
}

void merge(History& h,const Frame& frame) {
    const double size=h.cfg.size, q=size/4, broad=4*size;
    for (const auto& ray:frame.rays) {
        RayKey key={tile(ray.o[0],size),tile(ray.o[1],size),tile(ray.o[2],size),
                    tile(ray.p[0],q),tile(ray.p[1],q),tile(ray.p[2],q),ray.kind};
        auto pair=h.rays.emplace(key,ray);
        Ray* saved=&pair.first->second;
        if (!pair.second && saved->stamp>frame.stamp) continue;
        *saved=ray;
        if (ray.kind<0) h.columns[{tile(ray.p[0],size),tile(ray.p[1],size)}].hits.insert(saved);
        traverse(ray,broad,[&](Tile xy,double,double){h.index[xy].insert(saved);});
        h.index[{tile(ray.p[0],broad),tile(ray.p[1],broad)}].insert(saved);
    }
    for (const auto& item:frame.columns) {
        auto& col=h.columns[item.first];
        const auto& view=item.second;
        if (view.blocked && frame.stamp>=col.barrier && frame.stamp>col.cleared_at) {
            col.barrier=frame.stamp;
            col.floor_stamp=0;
            col.air.clear();
            col.clear=false;
            col.blocked=true;
        }
        if (frame.stamp<=col.barrier) continue;
        if (view.floor_stamp && view.floor_stamp>=col.floor_stamp) {
            col.floor=view.floor;
            col.floor_stamp=view.floor_stamp;
        }
        for (auto chunk:view.air) col.air[chunk.first]|=chunk.second;
        col.clear=clear_column(col,h.cfg);
        if (col.clear && col.blocked) {
            col.cleared_at=frame.stamp;
            col.blocked=false;
        }
    }
}

bool slab(double start,double delta,double low,double high,double& a,double& b) {
    if (!delta) return start>=low && start<high;
    double l=(low-start)/delta,r=(high-start)/delta;
    a=std::max(a,std::min(l,r)); b=std::min(b,std::max(l,r));
    return a<b;
}
// A later clear tile may contain the removed object and the floor beside it. Retire
// the old return only when newer measurements certify its own smaller XY patch.
bool active_hit(History& h,Ray& hit) {
    if (hit.superseded) return false;
    const double size=h.cfg.size,q=size/4,broad=4*size;
    Tile cell(tile(hit.p[0],size),tile(hit.p[1],size));
    auto column=h.columns.find(cell);
    if (column==h.columns.end() || column->second.cleared_at<=hit.stamp) return true;
    double x0=tile(hit.p[0],q)*q,y0=tile(hit.p[1],q)*q;
    auto bucket=h.index.find({tile(hit.p[0],broad),tile(hit.p[1],broad)});
    if (bucket==h.index.end()) return true;
    Column smaller;
    for (Ray* ray:bucket->second) {
        if (ray->stamp<=hit.stamp) continue;
        if (ray->kind==1 && ray->p[0]>=x0 && ray->p[0]<x0+q && ray->p[1]>=y0 && ray->p[1]<y0+q &&
            ray->stamp>=smaller.floor_stamp) {
            smaller.floor=ray->p[2];
            smaller.floor_stamp=ray->stamp;
        }
        double a=0.,b=1.;
        if (!slab(ray->o[0],ray->p[0]-ray->o[0],x0,x0+q,a,b) ||
            !slab(ray->o[1],ray->p[1]-ray->o[1],y0,y0+q,a,b)) continue;
        double z0=ray->o[2]+a*(ray->p[2]-ray->o[2]),z1=ray->o[2]+b*(ray->p[2]-ray->o[2]);
        int lo=tile(std::min(z0,z1),h.cfg.dz),hi=z0==z1 ? lo : int(std::ceil(std::max(z0,z1)/h.cfg.dz))-1;
        add_air(smaller.air,lo,hi);
    }
    hit.superseded=clear_column(smaller,h.cfg);
    return !hit.superseded;
}
struct Strip {
    std::map<Stamp,std::pair<double,double>> floors;
    std::vector<std::pair<double,double>> air;
};
const char* straight(History& h,double x,double y,double yaw,double length,double width,double distance,Stamp recent) {
    const double c=std::cos(yaw),s=std::sin(yaw),size=h.cfg.size,broad=4*size;
    if (recent>0) {
        // A new return beside the feet also vetoes translation, even when the front strip is clear.
        double rx=std::abs(c)*length+std::abs(s)*width,ry=std::abs(s)*length+std::abs(c)*width;
        for (int i=tile(x-rx,size);i<=tile(x+rx,size);++i) for (int j=tile(y-ry,size);j<=tile(y+ry,size);++j) {
            auto col=h.columns.find({i,j});
            if (col==h.columns.end()) continue;
            for (Ray* hit:col->second.hits) if (hit->stamp>=recent) {
                double dx=hit->p[0]-x,dy=hit->p[1]-y;
                if (std::abs(c*dx+s*dy)<=length && std::abs(c*dy-s*dx)<=width && active_hit(h,*hit))
                    return "blocked";
            }
        }
    }
    const double left=distance>=0 ? length : -length+distance, right=distance>=0 ? length+distance : -length;
    double xmin=INFINITY,xmax=-INFINITY,ymin=INFINITY,ymax=-INFINITY;
    for (double u:{left,right}) for (double v:{-width,width}) {
        double px=x+c*u-s*v,py=y+s*u+c*v;
        xmin=std::min(xmin,px); xmax=std::max(xmax,px); ymin=std::min(ymin,py); ymax=std::max(ymax,py);
    }
    int n=std::max(1,static_cast<int>(std::ceil(2*width/size-1e-9)));
    double dy=2*width/n;
    std::vector<Strip> strips(n);
    ++h.query;
    for (int i=tile(xmin,broad);i<=tile(xmax,broad);++i) for (int j=tile(ymin,broad);j<=tile(ymax,broad);++j) {
        auto bin=h.index.find({i,j});
        if (bin==h.index.end()) continue;
        for (Ray* ray:bin->second) {
            if (ray->visit==h.query) continue;
            ray->visit=h.query;
            const auto& p=ray->p;
            double u=c*(p[0]-x)+s*(p[1]-y),v=c*(p[1]-y)-s*(p[0]-x);
            if (u>=left && u<right && v>=-width && v<width) {
                if (ray->kind<0 && active_hit(h,*ray))
                    return "blocked";
                if (ray->kind==1) {
                    auto& floors=strips[std::min(n-1,static_cast<int>((v+width)/dy))].floors;
                    auto f=floors.emplace(ray->stamp,std::make_pair(p[2],p[2])).first;
                    f->second.first=std::min(f->second.first,p[2]);
                    f->second.second=std::max(f->second.second,p[2]);
                }
            }
            const auto& o=ray->o;
            double a=c*(o[0]-x)+s*(o[1]-y),b=c*(o[1]-y)-s*(o[0]-x);
            double lo=0.,hi=1.;
            if (!slab(a,u-a,left,right,lo,hi)) continue;
            double v0=b+(v-b)*lo,v1=b+(v-b)*hi;
            int first=std::max(0,tile(std::min(v0,v1)+width,dy));
            int last=std::min(n-1,tile(std::max(v0,v1)+width,dy));
            for (int col=first;col<=last;++col) {
                double enter=lo,leave=hi,bottom=-width+col*dy;
                if (!slab(b,v-b,bottom,bottom+dy,enter,leave) || leave-enter<=1e-8) continue;
                double z0=o[2]+(p[2]-o[2])*enter,z1=o[2]+(p[2]-o[2])*leave;
                strips[col].air.emplace_back(std::min(z0,z1),std::max(z0,z1));
            }
        }
    }
    for (auto& strip:strips) if (strip.floors.empty()) return "unknown";
    double previous=0.;
    for (int j=0;j<n;++j) {
        const auto& strip=strips[j];
        for (auto& f:strip.floors) if (f.second.second-f.second.first>h.cfg.step) return "blocked";
        double ground=strip.floors.rbegin()->second.first;
        if (j && std::abs(ground-previous)>std::min(h.cfg.step,h.cfg.slope*dy)) return "blocked";
        previous=ground;
    }
    for (const auto& strip:strips) {
        double ground=strip.floors.rbegin()->second.first;
        for (auto band:h.cfg.bands) {
            bool found=std::any_of(strip.air.begin(),strip.air.end(),[&](const std::pair<double,double>& interval){
                return interval.first<ground+band.second && interval.second>=ground+band.first;
            });
            if (!found) return "unknown";
        }
    }
    return "clear";
}

const char* area(History& h,const std::set<Tile>& tiles,bool hits_only) {
    bool missing=tiles.empty();
    for (auto xy:tiles) {
        auto it=h.columns.find(xy);
        if (it==h.columns.end()) { missing=true; continue; }
        if (it->second.blocked || std::any_of(it->second.hits.begin(),it->second.hits.end(),
                [&](Ray* ray){return active_hit(h,*ray);})) return "blocked";
        if (hits_only) continue;
        if (!it->second.clear) { missing=true; continue; }
        for (Tile adj:{Tile(xy.first+1,xy.second),Tile(xy.first,xy.second+1)}) {
            if (!tiles.count(adj)) continue;
            auto other=h.columns.find(adj);
            if (other!=h.columns.end() && other->second.clear &&
                std::abs(it->second.floor-other->second.floor)>std::min(h.cfg.step,h.cfg.slope*h.cfg.size))
                return "blocked";
        }
    }
    return missing && !hits_only ? "unknown" : "clear";
}

const char *map_name="uran.terrain.map", *frame_name="uran.terrain.frame";
template<class T> T* capsule(PyObject* obj,const char* name) {
    return static_cast<T*>(PyCapsule_GetPointer(obj,name));
}
void delete_map(PyObject* cap) { delete capsule<History>(cap,map_name); }
void delete_frame(PyObject* cap) { delete capsule<Frame>(cap,frame_name); }
bool coordinates(PyObject* seq,Point& p,Py_ssize_t n) {
    PyObject* row=PySequence_Fast(seq,"expected coordinates");
    if (!row) return false;
    bool ok=PySequence_Fast_GET_SIZE(row)==n;
    for (Py_ssize_t k=0;ok && k<n;++k) {
        p[k]=PyFloat_AsDouble(PySequence_Fast_GET_ITEM(row,k));
        ok=!PyErr_Occurred() && std::isfinite(p[k]);
    }
    Py_DECREF(row);
    if (!ok) PyErr_SetString(PyExc_ValueError,"expected finite coordinates");
    return ok;
}
PyObject* create_map(PyObject*,PyObject* args) {
    Config c{};
    if (!PyArg_ParseTuple(args,"ddddd",&c.size,&c.step,&c.height,&c.range,&c.slope)) return nullptr;
    c.dz=std::min({.01,c.size/10,(c.height-c.step)/8});
    double last=c.step;
    for (int i=1;i*c.size<c.height;++i) {
        double level=i*c.size;
        if (std::min(level-last,c.height-level)>=2*c.dz) {
            c.bands.emplace_back(last,level);
            last=level;
        }
    }
    c.bands.emplace_back(last,c.height);
    return PyCapsule_New(new History{c},map_name,delete_map);
}
PyObject* project_frame(PyObject*,PyObject* args) {
    PyObject *map,*origin,*ground,*points,*open;
    double bx,by; Stamp stamp;
    if (!PyArg_ParseTuple(args,"OOOddOOL",&map,&origin,&ground,&bx,&by,&points,&open,&stamp)) return nullptr;
    auto* h=capsule<History>(map,map_name);
    Point o,g;
    if (!h || !coordinates(origin,o,3) || !coordinates(ground,g,3)) return nullptr;
    std::vector<Point> samples;
    std::size_t returned=0;
    for (auto group:{points,open}) {
        PyObject* seq=PySequence_Fast(group,"terrain endpoints must be a sequence");
        if (!seq) return nullptr;
        for (Py_ssize_t i=0;i<PySequence_Fast_GET_SIZE(seq);++i) {
            Point p{};
            if (!coordinates(PySequence_Fast_GET_ITEM(seq,i),p,3)) { Py_DECREF(seq); return nullptr; }
            samples.push_back(p);
        }
        if (group==points) returned=samples.size();
        Py_DECREF(seq);
    }
    auto* frame=new Frame{stamp};
    if (std::hypot(o[0]-bx,o[1]-by)<=5.) {
        Py_BEGIN_ALLOW_THREADS
        for (std::size_t i=0;i<samples.size();++i) project(*frame,h->cfg,o,samples[i],g,bx,by,i<returned);
        Py_END_ALLOW_THREADS
    }
    return PyCapsule_New(frame,frame_name,delete_frame);
}
PyObject* merge_frame(PyObject*,PyObject* args) {
    PyObject *map,*frame;
    if (!PyArg_ParseTuple(args,"OO",&map,&frame)) return nullptr;
    auto* h=capsule<History>(map,map_name);
    auto* f=capsule<Frame>(frame,frame_name);
    if (!h || !f) return nullptr;
    merge(*h,*f);
    return PyBool_FromLong(!h->rays.empty());
}
PyObject* check_straight(PyObject*,PyObject* args) {
    PyObject* map; double x,y,yaw,l,w,d; Stamp recent=0;
    if (!PyArg_ParseTuple(args,"Odddddd|L",&map,&x,&y,&yaw,&l,&w,&d,&recent)) return nullptr;
    auto* h=capsule<History>(map,map_name);
    if (!h) return nullptr;
    return PyUnicode_FromString(straight(*h,x,y,yaw,l,w,d,recent));
}
PyObject* check_area(PyObject*,PyObject* args) {
    PyObject *map,*points; int hits_only;
    if (!PyArg_ParseTuple(args,"OOp",&map,&points,&hits_only)) return nullptr;
    auto* h=capsule<History>(map,map_name);
    if (!h) return nullptr;
    PyObject* seq=PySequence_Fast(points,"expected map tile coordinates");
    if (!seq) return nullptr;
    std::set<Tile> tiles;
    for (Py_ssize_t i=0;i<PySequence_Fast_GET_SIZE(seq);++i) {
        Tile xy;
        if (!PyArg_ParseTuple(PySequence_Fast_GET_ITEM(seq,i),"ii",&xy.first,&xy.second)) { Py_DECREF(seq); return nullptr; }
        tiles.insert(xy);
    }
    Py_DECREF(seq);
    return PyUnicode_FromString(area(*h,tiles,hits_only));
}
PyObject* check_cell(PyObject*,PyObject* args) {
    PyObject* map; int x,y; double r,pad;
    if (!PyArg_ParseTuple(args,"Oiidd",&map,&x,&y,&r,&pad)) return nullptr;
    auto* h=capsule<History>(map,map_name);
    if (!h) return nullptr;
    const double size=h->cfg.size;
    Tile centre(tile((x+.5)*r,size),tile((y+.5)*r,size));
    auto c=h->columns.find(centre);
    if (c==h->columns.end() || !c->second.clear) Py_RETURN_FALSE;
    int left=std::floor((x*r-pad)/size+1e-9),bottom=std::floor((y*r-pad)/size+1e-9);
    int right=std::ceil(((x+1)*r+pad)/size-1e-9),top=std::ceil(((y+1)*r+pad)/size-1e-9);
    // Test membership first; partial views should not build a full footprint for every cell.
    std::set<Tile> tiles;
    for (int i=left;i<right;++i) for (int j=bottom;j<top;++j) {
        auto it=h->columns.find({i,j});
        if (it==h->columns.end() || !it->second.clear) Py_RETURN_FALSE;
        tiles.emplace(i,j);
    }
    return PyBool_FromLong(area(*h,tiles,false)[0]=='c');
}
struct Beam { Point o; std::vector<Point> points; double bx,by; };
struct Label { Tile xy; bool drop=false, occupied=false, wall=false, ground=false, slope=false, clear=false; };
struct Surface { std::vector<double> hits; double low=0.; bool floor=false,drop=false,occupied=false,wall=false; };

std::vector<Label> classify(const std::vector<Beam>& beams,const Point& ground,double resolution,
                            double radius,double height,double step,double slope,double range,double sample) {
    int n=static_cast<int>(std::ceil(resolution/sample));
    double size=resolution/n;
    int padding=static_cast<int>(std::ceil(radius/size));
    std::map<Tile,Surface> surfaces;
    std::set<Tile> candidates;
    std::vector<Ray> rays;
    for (const auto& beam:beams) for (const auto& p:beam.points) {
        if (std::hypot(p[0]-beam.bx,p[1]-beam.by)>5.) continue;
        double dx=p[0]-beam.o[0],dy=p[1]-beam.o[1],dz=p[2]-beam.o[2];
        double d=std::sqrt(dx*dx+dy*dy+dz*dz);
        if (!(d>0 && d<=range)) continue;
        surfaces[{tile(p[0],size),tile(p[1],size)}].hits.push_back(p[2]);
        rays.push_back({beam.o,p,0,0});
    }
    for (auto& item:surfaces) {
        int x=item.first.first,y=item.first.second;
        auto& s=item.second;
        auto limits=std::minmax_element(s.hits.begin(),s.hits.end());
        s.low=*limits.first; s.wall=*limits.second-s.low>=height/2;
        double tolerance=step+slope*std::hypot((x+.5)*size-ground[0],(y+.5)*size-ground[1]);
        s.drop=s.low<ground[2]-tolerance;
        s.floor=!s.drop && s.low<=ground[2]+tolerance;
        if (s.floor) s.occupied=std::any_of(s.hits.begin(),s.hits.end(),[&](double z){return z-s.low>step && z-s.low<=height;});
        else s.occupied=!s.drop && s.low<=ground[2]+tolerance+height;
        for (int cx=tile(x-padding,n);cx<=tile(x+padding,n);++cx)
            for (int cy=tile(y-padding,n);cy<=tile(y+padding,n);++cy) candidates.emplace(cx,cy);
    }
    std::vector<Label> labels;
    bool needs_air=false;
    for (auto xy:candidates) {
        Label row{xy};
        int left=xy.first*n-padding,right=(xy.first+1)*n+padding;
        int bottom=xy.second*n-padding,top=(xy.second+1)*n+padding;
        int supported=0,walls=0; bool gentle=true;
        for (int x=left;x<right && !row.drop;++x) for (int y=bottom;y<top && !row.drop;++y) {
            auto it=surfaces.find({x,y});
            if (it==surfaces.end()) continue;
            const auto& s=it->second;
            row.drop=s.drop; row.occupied|=s.occupied; walls+=s.wall;
            if (!s.floor) continue;
            ++supported;
            for (Tile adj:{Tile(x+1,y),Tile(x,y+1)}) {
                if (adj.first>=right || adj.second>=top) continue;
                auto other=surfaces.find(adj);
                if (other==surfaces.end() || !other->second.floor) continue;
                double jump=std::abs(s.low-other->second.low);
                row.drop|=jump>step;
                gentle&=jump<=slope*size;
            }
        }
        row.wall=row.occupied && walls>=3;
        row.ground=!row.drop && !row.occupied && supported==(right-left)*(top-bottom);
        row.slope=row.ground && gentle; needs_air|=row.slope;
        labels.push_back(row);
    }
    if (!needs_air) return labels;
    std::set<std::array<int,3>> free;
    for (const auto& ray:rays) {
        const auto& o=ray.o;const auto& p=ray.p;
        double dx=p[0]-o[0],dy=p[1]-o[1],dz=p[2]-o[2];
        int count=static_cast<int>(std::ceil(2*std::sqrt(dx*dx+dy*dy+dz*dz)/size));
        std::array<int,3> end={tile(p[0],size),tile(p[1],size),tile(p[2],size)};
        for (int i=1;i<count;++i) {
            double t=double(i)/count;
            std::array<int,3> cell={tile(o[0]+dx*t,size),tile(o[1]+dy*t,size),tile(o[2]+dz*t,size)};
            if (cell!=end) free.insert(cell);
        }
    }
    for (auto& row:labels) if (row.slope) {
        row.clear=true;
        for (int x=row.xy.first*n-padding;x<(row.xy.first+1)*n+padding && row.clear;++x)
            for (int y=row.xy.second*n-padding;y<(row.xy.second+1)*n+padding && row.clear;++y) {
                double z=surfaces.at({x,y}).low;
                for (int k=std::max(tile(z,size)+1,tile(z+step,size));k<=tile(z+height,size);++k)
                    if (!free.count({x,y,k})) { row.clear=false; break; }
            }
    }
    return labels;
}

PyObject* classify_cells(PyObject*,PyObject* args) {
    PyObject *groups,*ground,*body;
    double r,radius,height,step,slope,range,sample;
    if (!PyArg_ParseTuple(args,"OOOddddddd",&groups,&ground,&body,&r,&radius,&height,&step,&slope,&range,&sample)) return nullptr;
    Point g{},b{};
    if (!coordinates(ground,g,3) || !coordinates(body,b,2)) return nullptr;
    std::vector<Beam> beams;
    PyObject* sequence=PySequence_Fast(groups,"expected beam groups");
    if (!sequence) return nullptr;
    for (Py_ssize_t j=0;j<PySequence_Fast_GET_SIZE(sequence);++j) {
        auto* item=PySequence_Fast_GET_ITEM(sequence,j);
        PyObject *origin=PyMapping_GetItemString(item,"origin"), *points=PyMapping_GetItemString(item,"points");
        PyObject* centre=PyDict_Check(item) ? PyDict_GetItemString(item,"body_xy") : nullptr;
        Beam beam{};Point xy=b;
        bool ok=origin && points && coordinates(origin,beam.o,3) && (!centre || coordinates(centre,xy,2));
        beam.bx=xy[0];beam.by=xy[1];
        if (ok) {
            auto* rows=PySequence_Fast(points,"expected beam endpoints");
            ok=rows!=nullptr;
            for (Py_ssize_t i=0;ok && i<PySequence_Fast_GET_SIZE(rows);++i) {
                Point p{};
                ok=coordinates(PySequence_Fast_GET_ITEM(rows,i),p,3);
                if (ok) beam.points.push_back(p);
            }
            Py_XDECREF(rows);
        }
        Py_XDECREF(origin); Py_XDECREF(points);
        if (!ok) { Py_DECREF(sequence); return nullptr; }
        beams.push_back(std::move(beam));
    }
    Py_DECREF(sequence);
    std::vector<Label> labels;
    Py_BEGIN_ALLOW_THREADS
    labels=classify(beams,g,r,radius,height,step,slope,range,sample);
    Py_END_ALLOW_THREADS
    PyObject* list=PyList_New(labels.size());
    if (!list) return nullptr;
    for (size_t i=0;i<labels.size();++i) {
        const auto& row=labels[i];
        PyObject* result=Py_BuildValue("{s:d,s:d,s:O}","x_m",(row.xy.first+.5)*r,"y_m",(row.xy.second+.5)*r,"observed",Py_True);
        if (!result) { Py_DECREF(list); return nullptr; }
        if (row.drop) PyDict_SetItemString(result,"step_ok",Py_False);
        else if (row.occupied) {
            PyDict_SetItemString(result,"occupied",Py_True);
            if (row.wall) PyDict_SetItemString(result,"vertical_surface",Py_True);
        } else if (row.ground) {
            PyDict_SetItemString(result,"ground_continuous",Py_True);
            PyDict_SetItemString(result,"step_ok",Py_True);
            PyDict_SetItemString(result,"slope_ok",row.slope ? Py_True : Py_False);
            if (row.clear) PyDict_SetItemString(result,"clearance_ok",Py_True);
        }
        PyList_SET_ITEM(list,i,result);
    }
    return list;
}

PyMethodDef methods[]={
    {"classify",classify_cells,METH_VARARGS,nullptr},{"create",create_map,METH_VARARGS,nullptr},{"project",project_frame,METH_VARARGS,nullptr},
    {"merge",merge_frame,METH_VARARGS,nullptr},{"straight",check_straight,METH_VARARGS,nullptr},
    {"area",check_area,METH_VARARGS,nullptr},{"cell",check_cell,METH_VARARGS,nullptr},{nullptr,nullptr,0,nullptr}
};
PyModuleDef module={PyModuleDef_HEAD_INIT,"_terrain_rays","Registered terrain measurements",-1,methods};
}
PyMODINIT_FUNC PyInit__terrain_rays() { return PyModule_Create(&module); }
