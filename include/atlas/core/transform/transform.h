#pragma once

#include <atlas/core/vectors/vec3.h>
#include <atlas/core/quaternion/quaternion.h>
#include <atlas/core/matrix/mat4.h>

#include <vector>
#include <cstdint>

namespace atlas::core::transform {
    class Transform {
    public:
        Transform();

        ~Transform();

        Transform(const Transform &) = delete;

        Transform &operator=(const Transform &) = delete;

        // setters

        void setLocalPosition(const vec::Vec3 &position);

        void setLocalRotation(const quat::Quat &rotation);

        void setLocalScale(const vec::Vec3 &scale);

        // getters

        const vec::Vec3 &getLocalPosition() const;

        const quat::Quat &getLocalRotation() const;

        const vec::Vec3 &getLocalScale() const;

        vec::Vec3 getWorldPosition();

        quat::Quat getWorldRotation();

        vec::Vec3 getWorldScale();

        // Matrix access

        void setLocalMatrix(const mat4::Mat4 &m);

        const mat4::Mat4 &getLocalMatrix();

        const mat4::Mat4 &getWorldMatrix();

        // Hierarchy

        void setParent(Transform *parent);

        std::size_t getChildCount() const;

        Transform *getChild(std::size_t index) const;

        void reorderChild(std::size_t from, std::size_t to);

        // Directions

        vec::Vec3 forward();

        vec::Vec3 up();

        vec::Vec3 right();

        // Space transforms

        vec::Vec3 transformPoint(const vec::Vec3 &localPoint);

        vec::Vec3 transformDirection(const vec::Vec3 &localDirection);

        vec::Vec3 inverseTransformPoint(const vec::Vec3 &worldPoint);

        vec::Vec3 inverseTransformDirection(const vec::Vec3 &worldDirection);

        // Optional user context
        void *ctx = nullptr;

    private:
        // Hierarchy helpers
        void addChild(Transform *child);

        void removeChild(const Transform *child);

        // Dirty handling
        void markDirty();

        // Lazy update
        void updateWorldMatrix();

    private:
        vec::Vec3 m_localPosition;
        quat::Quat m_localRotation;
        vec::Vec3 m_localScale;

        // Cached matrices
        mat4::Mat4 m_localMatrix;
        mat4::Mat4 m_worldMatrix;

        // Hierarchy
        Transform *m_parent;
        std::vector<Transform *> m_children;

        // Dirty flags
        bool m_localDirty;
        bool m_worldDirty;
    };
}
