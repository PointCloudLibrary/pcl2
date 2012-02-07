/** \file io.h 
 * \brief Declares functions for loading and saving .PCD files
 */

#include "pcl2/cloud.h"

namespace pcl2
{

/** \brief Load a PCD file 
 * \param filename The name of the PCD file to load
 * \return A Cloud containing the file's data
 */
Cloud loadCloud (const std::string & filename);


/** \brief Save a PCD file
 * \param filename The name of the PCD file to create
 * \param cloud The Cloud to save
 */
void saveCloud (const std::string & filename, const Cloud & cloud);

}
