require 'fileutils'
require 'open-uri'
require 'nokogiri'
require 'cgi'

class SdfDocumentationSource < Nanoc::DataSources::Filesystem
  identifier :sdf_documentation

  def initialize(site_config, items_repo, layouts_repo, code_snippets_repo)
    config_with_path = (config || {}).merge(content_dir: 'docs')
    super(site_config, items_repo, layouts_repo, config_with_path)
  end

  def up
    @knownVersions = YAML.safe_load_file('content/_data/versions.yaml')['knownVersions']
  end

  ###############################################
  # Parse markdown file before sending to redcarpet
  # @param[in] text Markdown text
  # @return Parsed markdown text
  def preParse(text)
    result = text.gsub(%r{^.*?<include\s.*?src=['"]http.*?['"].*?/>}){|part|
      match = part.match(%r{(.*)(<include.*)})
      spaces = match[1]

      # Get the source URL
      src = part.match(%r{src=['"].*?['"]})[0].sub(%r{src=['"](.*)['"]}, '\1')

      # Make sure we use https
      src.sub!("http://github", "https://github")

      # Open the source file
      begin
        data = URI.open(src, "r:UTF-8").read()
      rescue
      end

      if ! data.nil?
        # Set default start and length for the resluting include string
        startIndex = 0
        length = data.size()

        # Process the from="" portion of the <include>
        if !part.match(%r{from=.*?['"]\s}).nil?
          # Get the regexp
          frm = part.match(%r{from=.*?['"]\s})[0].sub(%r{from=['"](.*)['"]}, '\1')
          # Use ruby's Regexp to magically convert a string to a regexp
          reg = Regexp.new frm.slice(1, frm.size()-3)
          # Find the start index in the string
          startIndex = data.index(reg)

          # Saftey. If from was not found, then use zero
          if startIndex.nil?
            startIndex = 0
          end
        end

        # Process the to="" portion of the <include>
        if !part.match(%r{to=.*?['"]\s}).nil?
          to = part.match(%r{to=.*?['"]\s})[0].sub(%r{to=['"](.*)['"]}, '\1')
          reg = Regexp.new to.slice(1, to.size()-3)
          length = data.index(reg)

          # Saftey. If to was not found, then use data size
          if length.nil?
            length = data.size()
          else
            # Include the size of the matched to string.
            length += data.match(reg,length-1)[0].size + 1
          end
        end

        # Compute total length of string
        length = length - startIndex - 1

        # Get the string data to include
        data = data.slice(startIndex, length)

        # Set the correct html encoding.
        part = "\n~~~\n" + data + "\n~~~\n"
        part.gsub!(%{\n}, "\n#{spaces}")
      else
        part = "\n#{spaces}~~~\nError: Unable to access #{src}\n#{spaces}~~~\n"
      end
    }

    result = result.gsub(/%%%.*?%%%/m) {|part|
      data = part.match(/%%%(.*?)%%%/m)[1]
      part = "<pre class='nocopy'>" + CGI.escapeHTML(data) + "</pre>\n"
    }

    # Replace images
    return result.gsub(%r{\[\[file:.*?\]\]}) {|part|
      file = part.match(%r{file:(.*?)[\s*|\||\]]})[1]
      sizeMatch = part.match(%r{\|(.*?)\s*\]})
      if !sizeMatch.nil?
        size = sizeMatch[1]
      end

      # svg images hosted on github are shown as xml via the raw endpoint
      # use sanitize=true to display them as images
      # https://stackoverflow.com/a/16462143/7179595
      part = "<img src='#{file}'"

      # Add in size
      if !size.nil?
        part += " width='#{size}'"
      end
      part += "/>"
    }
  end

  def items
    manifest = Nokogiri::XML(File.read(File.join(@config[:content_dir], "manifest.xml")))
    new_items = []
    extra_files = []

    # Create a hash to store the category of each tutorial
    tutorial_categories = {}
    manifest.xpath("//content/categories/category").each do |cat|
      cat_ref = cat.attribute('ref').value
      cat.xpath("tutorials/tutorial").each do |tut|
        tutorial_categories[tut.text] = cat_ref
      end
    end

    all_tuts = []
    manifest.xpath("//content/tutorials/tutorial").each do |tut|
      raw_ref = tut.attribute('ref').value
      cat = tutorial_categories[raw_ref]
      ref = "#{cat}/#{raw_ref}"
      title = tut.attribute('title').value

      versions = tut.xpath('markdown').map do |md|
        v = md.attribute('version').value
        { version: v, path: md.text(), safe_version: v.gsub(/[>=<\s]/, '')}
      end

      all_tuts << {
        title: title,
        skill: tut.xpath('skill').text,
        desc: tut.xpath('description').text,
        ref: ref,
        cat: cat,
        type: tut.attribute('type') ? tut.attribute('type').value : "tutorial",
        versions: versions.map { |v| { num: v[:version], ref: v[:path], safe_version: v[:safe_version] } }
      }

      tut.xpath('markdown').each_with_index do |md, i|
        version = md.attribute('version').value
        safe_version = version.gsub(/[>=<\s]/, '')
        path = md.text()
        full_path = File.join(@config[:content_dir], path)
        raw_content = preParse(File.read(full_path))

        attributes = {
          title: title,
          description: tut.xpath('description').text.strip,
          skill: tut.xpath('skill').text,
          tags: tut.xpath('tags/tag').map(&:text),
          versions: versions.map { |v| {version: v[:version], safe_version: v[:safe_version]} },
          ref: ref,
          ver: version,
          safe_ver: safe_version,
          editFile: "https://github.com/gazebosim/sdf_tutorials/blob/master/docs/" + path,
          cat: cat,
          type: 'tutorial',
          content_layout: 'tutorial',
        }

        new_items << new_item(
          raw_content,
          attributes,
          "/#{ref}/#{safe_version}.md"
        )
        # Add all other files as binary
        dirname = File.dirname(full_path)
        extra_files << {ref: "#{ref}/#{safe_version}", basedir: dirname, files: Dir.glob(File.join(dirname,"**/*")).reject{ |f| f.end_with?(".md") or File.directory?(f) }}

        if i == versions.size - 1
          new_items << new_item(
            raw_content,
            attributes,
            "/#{ref}.md"
          )
        extra_files << {ref: ref, basedir: dirname, files: Dir.glob(File.join(dirname,"**/*")).reject{ |f| f.end_with?(".md") or File.directory?(f) }}
        end


      end
    end

    all_cats = []
    manifest.xpath('//categories/category').each do |cat|
      cat_ref = cat.attribute('ref').value
      cat_title = cat.attribute('title').value
      cat_skill = cat.attribute('skill').value

      all_cats << {
        title: cat_title,
        skill: cat_skill,
        ref: cat_ref,
      }
    end

    # Create category pages, and gather data for the main tutorials page
    current_cat_ref = 'specification' # default category
    current_cat = nil
    manifest.xpath('//categories/category').each do |cat|
      cat_ref = cat.attribute('ref').value
      cat_title = cat.attribute('title').value
      cat_desc = cat.xpath('description').text.strip.gsub(%r{\s+}, ' ')

      tutorials_in_cat = []
      cat.xpath('.//tutorials/tutorial').each do |tut_ref|
        tut = all_tuts.find { |t| t[:ref] == "#{cat_ref}/#{tut_ref.text}"}
        if tut
          tutorials_in_cat << tut
        end
      end

      attributes = {
        title: cat_title,
        description: cat_desc,
        tutorials: tutorials_in_cat,
        type: 'tutorial_list',
        content_layout: 'show',
        cats: all_cats.map { |c| c.merge({active: c[:ref] == cat_ref ? "active" : ""}) },
        current: {
          ref: cat_ref,
          title: cat_title,
          desc: cat_desc,
          tutorials: tutorials_in_cat
        },
        tuts: all_tuts
      }

      new_items << new_item(
        '',
        attributes,
        "/#{cat_ref}.md"
      )

      if cat_ref == current_cat_ref
        current_cat = attributes
      end
    end

    new_items << new_item(
      '',
      current_cat.merge({
        title: 'Tutorial Categories',
        type: 'category_list',
        content_layout: 'show'
      }),
      '/index.md'
    )

    extra_files.each do |ef|
      ef[:files].each do |file|
        new_items << new_item(File.absolute_path(file), {}, "/#{File.join(ef[:ref], file.delete_prefix(ef[:basedir]))}", binary: true)
      end
    end
    new_items
  end
end
