#include <atlas/core/transform/transform.h>

namespace atlas::core::transform {
#pragma region constructors

    // Constructors

    Transform::Transform()
        : m_localPosition(0.0f, 0.0f, 0.0f)
          , m_localRotation()
          , m_localScale(1.0f, 1.0f, 1.0f)
          , m_localMatrix(mat4::identity())
          , m_worldMatrix(mat4::identity())
          , m_parent(nullptr)
          , m_localDirty(true)
          , m_worldDirty(true) {
    }

    Transform::~Transform() {
        setParent(nullptr);
    }

#pragma endregion

#pragma region setters

    // Local setters

    void Transform::setLocalPosition(const vec::Vec3 &position) {
        m_localPosition = position;
        m_localDirty = true;
        markDirty();
    }

    void Transform::setLocalRotation(const quat::Quat &rotation) {
        m_localRotation = rotation;
        m_localDirty = true;
        markDirty();
    }

    void Transform::setLocalScale(const vec::Vec3 &scale) {
        m_localScale = scale;
        m_localDirty = true;
        markDirty();
    }

#pragma endregion

#pragma region getters

    // Local getters

    const vec::Vec3 &Transform::getLocalPosition() const {
        return m_localPosition;
    }

    const quat::Quat &Transform::getLocalRotation() const {
        return m_localRotation;
    }

    const vec::Vec3 &Transform::getLocalScale() const {
        return m_localScale;
    }

    // World getters

    vec::Vec3 Transform::getWorldPosition() {
        updateWorldMatrix();
        return mat4::getTranslation(m_worldMatrix);
    }

    quat::Quat Transform::getWorldRotation() {
        updateWorldMatrix();
        return mat4::getRotation(m_worldMatrix);
    }

    vec::Vec3 Transform::getWorldScale() {
        updateWorldMatrix();
        return mat4::getScale(m_worldMatrix);
    }

#pragma endregion

#pragma region translate

    void Transform::translateLocal(const vec::Vec3 &delta) {
        m_localPosition += m_localRotation * delta;

        m_localDirty = true;
        markDirty();
    }

    void Transform::translateLocal(float x, float y, float z) {
        translateLocal(vec::Vec3(x, y, z));
    }

    void Transform::translateWorld(const vec::Vec3 &delta) {
        vec::Vec3 worldPos = getWorldPosition() + delta;

        if (m_parent)
            setLocalPosition(mat4::transformPoint(mat4::inverseTRS(m_parent->getWorldMatrix()), worldPos));
        else
            setLocalPosition(worldPos);
    }

    void Transform::translateWorld(float x, float y, float z) {
        translateWorld(vec::Vec3(x, y, z));
    }

#pragma endregion

#pragma region rotate

    void Transform::rotateLocal(const quat::Quat &delta) {
        setLocalRotation((m_localRotation * delta).normalized());
    }

    void Transform::rotateLocal(const vec::Vec3 &eulerRad) {
        rotateLocal(quat::fromEuler(eulerRad));
    }

    void Transform::rotateLocal(float x, float y, float z) {
        rotateLocal(vec::Vec3{x, y, z});
    }

    void Transform::rotateWorld(const quat::Quat &delta) {
        // Apply world-space rotation
        quat::Quat newWorldRot = delta * getWorldRotation();

        if (m_parent)
            setLocalRotation((quat::inverse(m_parent->getWorldRotation()) * newWorldRot).normalized());
        else
            setLocalRotation(newWorldRot);
    }

    void Transform::rotateWorld(const vec::Vec3 &eulerRad) {
        rotateWorld(quat::fromEuler(eulerRad));
    }

    void Transform::rotateWorld(float x, float y, float z) {
        rotateWorld(quat::fromEuler({x, y, z}));
    }

#pragma endregion

#pragma region matrix access

    // Matrix access

    void Transform::setLocalMatrix(const mat4::Mat4 &m) {
        m_localMatrix = m;

        mat4::decomposeTRS(m, m_localPosition, m_localRotation, m_localScale);

        m_localDirty = false;
        markDirty();
    }

    const mat4::Mat4 &Transform::getLocalMatrix() {
        if (m_localDirty) {
            m_localMatrix = mat4::TRS(m_localPosition, m_localRotation, m_localScale);

            m_localDirty = false;
        }

        return m_localMatrix;
    }

    const mat4::Mat4 &Transform::getWorldMatrix() {
        updateWorldMatrix();
        return m_worldMatrix;
    }

#pragma endregion

#pragma region hierarchy

    // Hierarchy

    void Transform::setParent(Transform *parent) {
        if (m_parent == parent)
            return;

        updateWorldMatrix();

        if (m_parent)
            m_parent->removeChild(this);

        m_parent = parent;

        if (m_parent) {
            m_parent->addChild(this);

            m_parent->updateWorldMatrix();

            // local = inverse(parentWorld) * world
            mat4::Mat4 local = mat4::mul(mat4::inverseTRS(m_parent->m_worldMatrix), m_worldMatrix);
            setLocalMatrix(local);
        } else {
            setLocalMatrix(m_worldMatrix);
        }
    }

    std::size_t Transform::getChildCount() const {
        return m_children.size();
    }

    Transform *Transform::getChild(std::size_t index) const {
        return index < m_children.size() ? m_children[index] : nullptr;
    }

    void Transform::reorderChild(std::size_t from, std::size_t to) {
        if (from >= m_children.size() || to >= m_children.size() || from == to)
            return;

        Transform *child = m_children[from];
        m_children.erase(m_children.begin() + from);
        m_children.insert(m_children.begin() + to, child);
    }

#pragma endregion

#pragma region directions

    // Directions

    vec::Vec3 Transform::forward() {
        return transformDirection(vec::Vec3(0.0f, 0.0f, 1.0f));
    }

    vec::Vec3 Transform::up() {
        return transformDirection(vec::Vec3(0.0f, 1.0f, 0.0f));
    }

    vec::Vec3 Transform::right() {
        return transformDirection(vec::Vec3(1.0f, 0.0f, 0.0f));
    }

#pragma endregion

#pragma region space conversion

    // Space transforms

    vec::Vec3 Transform::transformPoint(const vec::Vec3 &localPoint) {
        updateWorldMatrix();
        return mat4::transformPoint(m_worldMatrix, localPoint);
    }

    vec::Vec3 Transform::transformDirection(const vec::Vec3 &localDirection) {
        updateWorldMatrix();
        return mat4::transformDirection(m_worldMatrix, localDirection);
    }

    vec::Vec3 Transform::inverseTransformPoint(const vec::Vec3 &worldPoint) {
        updateWorldMatrix();
        return mat4::transformPoint(mat4::inverseTRS(m_worldMatrix), worldPoint);
    }

    vec::Vec3 Transform::inverseTransformDirection(const vec::Vec3 &worldDirection) {
        updateWorldMatrix();
        return mat4::transformDirection(mat4::inverseTRS(m_worldMatrix), worldDirection);
    }

#pragma endregion

#pragma region utility

    // Internal helpers

    void Transform::addChild(Transform *child) {
        m_children.push_back(child);
    }

    void Transform::removeChild(const Transform *child) {
        for (std::size_t i = 0; i < m_children.size(); ++i) {
            if (m_children[i] == child) {
                m_children.erase(m_children.begin() + i);
                return;
            }
        }
    }

    void Transform::markDirty() {
        m_worldDirty = true;

        for (Transform *child: m_children)
            child->markDirty();
    }

    void Transform::updateWorldMatrix() {
        if (!m_worldDirty)
            return;

        const mat4::Mat4 &local = getLocalMatrix();

        if (m_parent) {
            m_parent->updateWorldMatrix();
            m_worldMatrix = mat4::mul(m_parent->m_worldMatrix, local);
        } else {
            m_worldMatrix = local;
        }

        m_worldDirty = false;
    }

#pragma endregion
}
