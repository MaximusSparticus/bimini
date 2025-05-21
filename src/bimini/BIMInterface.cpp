#include "bimini/BIMInterface.hpp"

#include <format>
#include <iostream>
#include <sstream>

#include <ifcpp/IFC4X3/EntityFactory.h>
#include <ifcpp/model/BuildingModel.h>
#include <ifcpp/reader/ReaderSTEP.h>
#include <ifcpp/writer/WriterSTEP.h>

namespace bimini {

BIMInterface::BIMInterface()
: m_model(nullptr)
, m_ifcPath() {
}

bool BIMInterface::loadIFC(const std::filesystem::path& path) {
    constexpr bool success = true;

    if (!std::filesystem::exists(path)) {
        return !success;
    }

    if (!m_model) {
        m_model.reset();
        // TODO(zmd): cleanup?
    }

    m_model = std::make_shared<BuildingModel>();
    
    // Create a reader
    auto reader = std::make_shared<ReaderSTEP>();
    
    // Read the IFC file
    reader->loadModelFromFile(path.string(), m_model);
    m_model->resolveInverseAttributes();

    return success;
}

void BIMInterface::dumpEntities() const {
    if (!m_model) return;
    const auto &entities = m_model->getMapIfcEntities();

    for (const auto &pair : entities) {
        //std::ostringstream ss;
        //ss << "Entity ID" << pair.first << std::endl;
        std::cout << std::format("Entity ID: {}\tClass ID: {}\tClass Name: {}\n",
            pair.first, pair.second->classID(), IFC4X3::EntityFactory::getStringForClassID(pair.second->classID()));
    }

}

}   // namespace bimini
