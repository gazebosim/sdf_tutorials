require 'fileutils'
require 'nokogiri'
require 'open-uri'

def tabs
  %w[
    sdf
    world
    scene
    state
    physics
    light
    actor
    model
    link
    sensor
    joint
    collision
    visual
    material
    geometry
  ]
end

class SdfSpecDataSource < ::Nanoc::DataSource
  identifier :sdf_spec

  def up
    @knownVersions = YAML.safe_load_file('content/_data/versions.yaml')['knownVersions']
    @defaultVersion = YAML.safe_load_file('content/_data/versions.yaml')['defaultVersion']

    external_data_dir = '.external_data/sdf_spec'
    if !Dir.exist?(external_data_dir) || Dir.empty?(external_data_dir)
      puts "Fetching sdf specs..."
      FileUtils.rm_rf(external_data_dir) if Dir.exist?(external_data_dir)
      FileUtils.mkdir_p(external_data_dir)
      Dir.chdir(external_data_dir) do
        @knownVersions.each do |version|
          # Description files have the format `full_<major>-<minor>.sdf`. eg. full_1-7.sdf
          desc_file = "full_#{version.gsub('.', '-')}.sdf"
          puts "Fetching #{desc_file}"
          document = URI.open("https://osrf-distributions.s3.amazonaws.com/sdformat/api/#{desc_file}")
          IO.copy_stream(document, desc_file)
        end
      end
    end

    @cache_dir = 'tmp/sdf_spec_cache'
    FileUtils.rm_rf(@cache_dir) if Dir.exist?(@cache_dir)
    FileUtils.mkdir_p(@cache_dir)

    @knownVersions.each do |version|
      desc_file = "full_#{version.gsub('.', '-')}.sdf"
      document = Nokogiri::XML(File.read(File.join(external_data_dir, desc_file)))

      tabs.each do |tab|
        template = Nokogiri::XSLT(File.read(File.join('content', '_spec_files', "#{tab}.xsl")))
        content = template.transform(document).to_s.gsub(/__VERSION__/, version)

        File.open(File.join(@cache_dir, "#{version}-#{tab}.html"), 'w') do |file|
          file.write(content)
        end
      end
    end
  end

  def item_changes
    []
  end

  def layout_changes
    []
  end

  def items
    specs = @knownVersions.map do |version|
      items = []
      tabs.each.with_index do |tab, index|
        example = CGI.escapeHTML(File.read(File.join('content', '_spec_files', "#{tab}_example.xml"))).gsub(
          /__VERSION__/, version
        )

        content = File.read(File.join(@cache_dir, "#{version}-#{tab}.html"))
        attributes = {
          title: "Spec for version #{version}",
          tab: tab,
          version: version,
          example: example
        }
        items.push(new_item(content, attributes, Nanoc::Identifier.new("/#{version}/#{tab}.spec")))

        if index == 0
          items.push(new_item(content, attributes, Nanoc::Identifier.new("/#{version}/index.spec")))
        end
        if version == @defaultVersion and index == 0
          items.push(new_item(content, attributes, Nanoc::Identifier.new("/index.spec")))
        end
      end
      items
    end

    specs.flatten
  end
end
