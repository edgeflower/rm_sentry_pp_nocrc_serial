#pragma once

// DEPRECATED: Include chiral_endpoint.hpp instead.
// TalosDataWriter → talos::chiral::ipc::ChannelWriter<TalosData>
// TalosDataReader → talos::chiral::ipc::ChannelReader<TalosData>
#include "chiral_endpoint.hpp"

#include <cstddef>

namespace talos::chiral::ipc {

inline constexpr const char* TALOS_SHM_NAME = ShmName<talos::chrial::TalosData>::value;
inline constexpr size_t TALOS_SHM_SIZE      = sizeof(ShmLayout<talos::chrial::TalosData>);

} // namespace talos::chiral::ipc
