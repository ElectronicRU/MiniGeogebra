#ifndef DATAMODEL_H
#define DATAMODEL_H
#include <cassert>
#include <cstddef>
#include <forward_list>
#include <optional>
#include <vector>


struct ShapeIndex {
    size_t index;

    ShapeIndex(size_t index) : index(index) {}
};

struct Point {
    float x, y;
    Point(float x, float y): x(x), y(y) {}
};

struct ControlPoint : public Point {
    std::optional<ShapeIndex> referencingShape;
    std::optional<ShapeIndex> referencingShape2;
    std::forward_list<ShapeIndex> referencingShapeMore;

    ControlPoint(Point&& p): Point(p) {}

    void addReference(ShapeIndex index) {
        if (!referencingShape.has_value()) {
            referencingShape = index;
        } else if (!referencingShape2.has_value()) {
            referencingShape2 = index;
        } else {
            referencingShapeMore.push_front(index);
        }
    }

    template<typename L> void remove_if(L&& callable) {
        if (referencingShape.has_value() && callable(referencingShape.value())) {
            referencingShape.reset();
        }
        if (referencingShape2.has_value() && callable(referencingShape2.value())) {
            referencingShape2.reset();
        }

        referencingShapeMore.remove_if(callable);
    }
};

class ControlPointArena;

struct ControlPointRef {
    const size_t index;
    Point cached;

    ControlPointRef(ControlPointArena const& arena, size_t index) :
        index(index), cached(arena[index]) {}

    void update(ControlPointArena const& arena) {
        cached = arena[index];
    }
};


class ControlPointArena {
    std::vector<ControlPoint> innerStorage{};
    size_t tempWaterMark{};

    void deallocTemporary(size_t index) {
        assert(index == innerStorage.size() - 1);
        assert(index >= tempWaterMark);
        innerStorage.pop_back();
    }

public:
    class TemporaryControlPointRef : public ControlPointRef {
        friend class ControlPointArena;
        ControlPointArena& arena;
        TemporaryControlPointRef(ControlPointArena& arena, size_t index) :
            ControlPointRef(arena, index), arena(arena) {}
    public:
        void update() {
            ControlPointRef::update(arena);
        }
        ~TemporaryControlPointRef() {
            arena.deallocTemporary(index);
        }
    };

    [[nodiscard]] TemporaryControlPointRef allocateTemporary(Point point) {
        size_t index = innerStorage.size();
        innerStorage.emplace_back(point);
        return TemporaryControlPointRef(*this, index);
    }

    [[nodiscard]] ControlPointRef allocate(Point point) {
        assert(tempWaterMark == innerStorage.size());
        size_t index = innerStorage.size();
        ++tempWaterMark;
        innerStorage.emplace_back(point);
        return ControlPointRef(*this, index);
    }

    const Point& operator[](size_t index) const {
        return innerStorage[index];
    }
};

struct BoundingBox {
    float min_x, max_x, min_y, max_y;
    BoundingBox(float min_x, float max_x, float min_y, float max_y) : min_x(min_x), max_x(max_x), min_y(min_y), max_y(max_y) {}
};

struct Quadratic {
    float x_squared;
    float xy_half, y_squared;
    float x_half, y_half, free_term;

    Point closeZero(Point) const;
    bool frameHasZeros(BoundingBox const& bbox) const;
};

class Shape {
protected:
    std::vector<ControlPointRef> controlPoints;
    std::optional<BoundingBox> bbox;
    Quadratic approximateDistance;
    Shape(std::vector<ControlPointRef>&& controlPoints, std::optional<BoundingBox>&& bbox, Quadratic&& approximateDistance) :
        controlPoints(std::move(controlPoints)), bbox(bbox), approximateDistance(approximateDistance) {}
public:
    virtual bool intersects(BoundingBox const& bbox) const;
    void update(ControlPointArena& arena);
};

template<typename T>
class ShapeBuilder {
    static_assert(std::is_base_of_v<Shape, T>);
    static_assert(std::is_same_v<size_t, decltype(T::MIN_CONTROL_POINTS)>);
    static_assert(std::is_same_v<size_t, decltype(T::MAX_CONTROL_POINTS)>);

    std::vector<ControlPointRef> controlPoints;
    std::optional<ControlPointArena::TemporaryControlPointRef> tempCtlCursor;
    std::optional<ControlPointArena::TemporaryControlPointRef> tempCtlSnapped;

    ControlPointArena& arena;

public:
    enum class Snap {
        NoSnap,
        Snap
    };

    ShapeBuilder(ControlPointArena& arena) : arena(arena) {}

    bool isFinished() {
        return controlPoints.size() >= T::MIN_CONTROL_POINTS;
    }
    bool canTakeMorePoints() {
        return controlPoints.size() < T::MAX_CONTROL_POINTS;
    }
    bool hasTemporary() {
        return tempCtlCursor.has_value();
    }
    void promoteTemporary(Snap snap) {
        assert(hasTemporary());
        size_t idx = tempCtlCursor.value().index;
        switch(snap) {
        case Snap::NoSnap:
            break;
        case Snap::Snap:
            if (tempCtlSnapped.has_value()) {
                idx = tempCtlSnapped.value().index;
            }
            break;
        }
        Point p = arena[idx];
        tempCtlSnapped.reset();
        tempCtlCursor.reset();
        controlPoints.push_back(arena.allocate(p));
    }


};


class DataModel
{
public:
    DataModel();
};

#endif // DATAMODEL_H
