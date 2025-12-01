def versions_data(key)
  @items['/_data/versions.yaml'][key]
end

def versions
  versions_data :versions
end

def releaseVersion
  versions_data :releaseVersion
end

def releaseDate
  versions_data :releaseDate
end

def releaseTar
  versions_data :releaseTar
end

def knownVersions
  versions_data :knownVersions
end

def defaultVersion
  versions_data :defaultVersion
end

