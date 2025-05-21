/* Copyright 2018-2019 TomTom N.V.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */
#pragma once

#include <string>
#include <map>
#include <QtNetwork>
#include "tile.hpp"

namespace rviz_satellite
{

class tile_request_error : public std::exception
{
private:
  std::string message_;

public:
  explicit tile_request_error(const std::string & message)
  : message_(message)
  {
  }

  const char * what() const noexcept override
  {
    return message_.c_str();
  }
};

/**
 * @brief Download slippy tiles from a Tile server.
 */
class TileClient : public QObject
{
  Q_OBJECT

private:
  QNetworkAccessManager * manager_;
  std::map<TileId, std::promise<QImage>> tile_promises_;
  std::string session_token_; ///< Stores the session token for Google Maps API requests

public:
  TileClient();

  /**
   * @brief Load a specific tile
   *
   * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached. Otherwise they
   * get downloaded.
   *
   * If server url contains "file://", local filesystem will be used.
   *
   */
  std::future<QImage> request(const TileId & tile_id);

  /**
   * @brief Load a specific tile from filesystem
   *
   */
  std::future<QImage> request_local(const TileId & tile_id);

  /**
   * @brief Load a specific tile from internet
   *
   * Since QNetworkDiskCache is used, tiles will be loaded from the file system if they have been cached. Otherwise they
   * get downloaded.
   */
  std::future<QImage> request_remote(const TileId & tile_id);

  std::string createGoogleMapsSession();
  std::string extractSessionToken(const QByteArray &response);
  std::string googleMapsTileURL(const TileId &tile_id, std::string const &session_token);

private Q_SLOTS:
  void request_finished(QNetworkReply * reply);
};

}  // namespace rviz_satellite
