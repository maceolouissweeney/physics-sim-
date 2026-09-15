#pragma once

// Shared helpers for the JNI glue (jni_*.cpp).
//
// Rules (see CLAUDE.md §8): the JNI glue calls only the C ABI, never C++ internals, so an FFM binding can
// reuse the same surface. Failures become Java exceptions; nothing here may crash the JVM on bad input.
// Arrays are copied (Get*ArrayRegion) rather than pinned: these calls happen at setup time, not per tick.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <jni.h>

#include <frcsim/frcsim_c.h>

namespace frcsim::jni {

inline void throwJava(JNIEnv* env, const char* className, const char* message) {
    jclass cls = env->FindClass(className);
    if (cls != nullptr) {
        env->ThrowNew(cls, message);
        env->DeleteLocalRef(cls);
    }
}

/// Throws the Java exception matching a failed status. Returns true if an exception was thrown.
inline bool check(JNIEnv* env, frcsim_status status) {
    if (status == FRCSIM_OK) {
        return false;
    }
    const char* message = frcsim_last_error();
    if (message == nullptr || message[0] == '\0') {
        message = frcsim_status_string(status);
    }
    switch (status) {
    case FRCSIM_ERR_INVALID_ARGUMENT:
        throwJava(env, "java/lang/IllegalArgumentException", message);
        break;
    case FRCSIM_ERR_OUT_OF_MEMORY:
        throwJava(env, "java/lang/OutOfMemoryError", message);
        break;
    case FRCSIM_ERR_CAPACITY_EXCEEDED:
        throwJava(env, "org/frcsim/CapacityExceededException", message);
        break;
    case FRCSIM_ERR_NOT_FOUND:
        throwJava(env, "java/util/NoSuchElementException", message);
        break;
    default:
        throwJava(env, "org/frcsim/FrcSimException", message);
        break;
    }
    return true;
}

inline frcsim_world* worldFromHandle(JNIEnv* env, jlong handle) {
    if (handle == 0) {
        throwJava(env, "java/lang/IllegalStateException", "world has been closed");
        return nullptr;
    }
    return reinterpret_cast<frcsim_world*>(static_cast<intptr_t>(handle));
}

/// Modified-UTF-8 view of a Java string (fine for identifiers; use byte[] for arbitrary text).
class JavaString {
public:
    JavaString(JNIEnv* env, jstring string)
        : m_env(env), m_string(string), m_chars(string != nullptr ? env->GetStringUTFChars(string, nullptr) : nullptr) {
        if (string == nullptr) {
            throwJava(env, "java/lang/NullPointerException", "string argument must not be null");
        }
    }
    ~JavaString() {
        if (m_chars != nullptr) {
            m_env->ReleaseStringUTFChars(m_string, m_chars);
        }
    }
    JavaString(const JavaString&) = delete;
    JavaString& operator=(const JavaString&) = delete;

    [[nodiscard]] const char* get() const { return m_chars; }

private:
    JNIEnv* m_env;
    jstring m_string;
    const char* m_chars;
};

inline std::vector<float> copyFloats(JNIEnv* env, jfloatArray array) {
    std::vector<float> values;
    if (array != nullptr) {
        values.resize(static_cast<std::size_t>(env->GetArrayLength(array)));
        env->GetFloatArrayRegion(array, 0, static_cast<jsize>(values.size()), values.data());
    }
    return values;
}

inline jintArray toJavaIntArray(JNIEnv* env, const jint* values, std::size_t count) {
    const auto n = static_cast<jsize>(count);
    jintArray result = env->NewIntArray(n);
    if (result != nullptr) {
        env->SetIntArrayRegion(result, 0, n, values);
    }
    return result;
}

inline jint toJavaIndex(JNIEnv* env, frcsim_status status, std::uint32_t index) {
    return check(env, status) ? -1 : static_cast<jint>(index);
}

} // namespace frcsim::jni
