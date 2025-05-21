#ifndef INCLUDE_BIMINI_BIM_INTERFACE_HPP_
#define INCLUDE_BIMINI_BIM_INTERFACE_HPP_

#include <filesystem>
#include <memory>

class BuildingModel;

namespace bimini {

class BIMInterface final {
 public:
    /**
     * @brief default constructor
     */
    BIMInterface();

    /**
     * @brief load the IFC entities into the BuildingModel object
     * @param path is the path to the IFC file
     */
    bool loadIFC(const std::filesystem::path& path);

    /**
     * @brief dump the entities to console
     */
    void dumpEntities() const;

 private:
    std::shared_ptr<BuildingModel> m_model;
    std::filesystem::path m_ifcPath;
};

}   // namespace bimini

#endif  // INCLUDE_BIMINI_BIM_INTERFACE_HPP_
