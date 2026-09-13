#include <gtest/gtest.h>

#include "world/body_tag.h"

namespace frcsim {
namespace {

TEST(BodyTag, RoundTrips) {
    const BodyTag tag{BodyKind::Piece, 17, 123456};
    const BodyTag decoded = BodyTag::decode(tag.encode());
    EXPECT_EQ(decoded.kind, BodyKind::Piece);
    EXPECT_EQ(decoded.material, 17);
    EXPECT_EQ(decoded.index, 123456u);
    EXPECT_EQ(BodyTag::decodeMaterial(tag.encode()), 17);
}

TEST(BodyTag, ExtremesDoNotOverlap) {
    const BodyTag tag{BodyKind::Kinematic, 0xFF, 0xFFFFFFFFu};
    const BodyTag decoded = BodyTag::decode(tag.encode());
    EXPECT_EQ(decoded.kind, BodyKind::Kinematic);
    EXPECT_EQ(decoded.material, 0xFF);
    EXPECT_EQ(decoded.index, 0xFFFFFFFFu);
}

TEST(BodyTag, ZeroUserDataIsNoneWithDefaultMaterial) {
    const BodyTag decoded = BodyTag::decode(0);
    EXPECT_EQ(decoded.kind, BodyKind::None);
    EXPECT_EQ(decoded.material, 0);
    EXPECT_EQ(decoded.index, 0u);
}

} // namespace
} // namespace frcsim
