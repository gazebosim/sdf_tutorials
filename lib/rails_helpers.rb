def favicon_link_tag(icon)
  "<link rel='icon' href='/assets/images/#{icon}' type='image/x-icon'/>"
end

def asset_path(asset)
  "/assets/#{asset}"
end
