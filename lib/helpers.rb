require 'redcarpet'
use_helper Nanoc::Helpers::Rendering
use_helper Nanoc::Helpers::LinkTo
use_helper Nanoc::Helpers::Text

def table_of_contents(item)
  renderer = Redcarpet::Render::HTML_TOC.new(nesting_level: 4)
  markdown = Redcarpet::Markdown.new(renderer)
  markdown.render(item.raw_content)
end

def skillToColor(skill)
  if skill == "beginner"
    return 'success'
  elsif skill == "intermediate"
    return 'info'
  else
    return 'warning'
  end
end

def markdown
  @markdown ||= Redcarpet::Markdown.new(Redcarpet::Render::HTML.new(prettify: true, with_toc_data: true, safe_links_only: true),
                                        autolink: true, tables: true,
                                        strikethrough: true,
                                        fenced_code_blocks: true)
end

def markdown_toc
  @markdown_toc ||= Redcarpet::Markdown.new(Redcarpet::Render::HTML_TOC.new(nesting_level: 4),
                                            autolink: true, tables: true,
                                            strikethrough: true,
                                            fenced_code_blocks: true)
end
